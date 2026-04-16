#include "TurntableController.h"
#include "RobotSlaveDriver.h"

#include <cmath>
#include <cstdio>

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------
TurntableController::TurntableController(RobotSlaveDriver& drive,
                                         int32_t encoder_counts_per_rev)
    : drive_(drive)
    , encoder_counts_per_rev_(encoder_counts_per_rev)
{
}

// ---------------------------------------------------------------------------
// Coordinate conversion
// ---------------------------------------------------------------------------
int32_t TurntableController::deg_to_counts(double deg) const
{
    return static_cast<int32_t>(
        deg / 360.0 * static_cast<double>(encoder_counts_per_rev_));
}

double TurntableController::counts_to_deg(int32_t c) const
{
    return static_cast<double>(c) / static_cast<double>(encoder_counts_per_rev_)
           * 360.0;
}

int32_t TurntableController::station_counts(Station s) const
{
    return deg_to_counts(static_cast<double>(static_cast<uint8_t>(s)) * 120.0);
}

// ---------------------------------------------------------------------------
// S-curve profile  (normalized 0..1 over duration T)
//   5th-order polynomial: s(t) = 10*(t/T)^3 - 15*(t/T)^4 + 6*(t/T)^5
//   Gives zero velocity and zero acceleration at endpoints.
// ---------------------------------------------------------------------------
double TurntableController::s_curve_profile(double t, double T) const
{
    if (t <= 0.0) return 0.0;
    if (t >= T)   return 1.0;
    double tau = t / T;
    double tau3 = tau * tau * tau;
    return tau3 * (10.0 - 15.0 * tau + 6.0 * tau * tau);
}

// ---------------------------------------------------------------------------
// generate_trajectory  --  compute trajectory duration from motion limits
// ---------------------------------------------------------------------------
void TurntableController::generate_trajectory()
{
    double dist_deg = std::abs(
        counts_to_deg(traj_target_ - traj_start_));

    // For 5th-order polynomial S-curve, peak velocity = 1.875 * avg_velocity.
    // Minimum time = distance / (peak_velocity_allowable / 1.875).
    double t_vel = dist_deg * 1.875 / max_vel_;  // Time limited by velocity
    double t_acc = std::sqrt(5.7735 * dist_deg / max_acc_);  // Time limited by acceleration (5th order)
    traj_duration_ = std::max(t_vel, t_acc) * 1.1;  // 10% safety margin

    // Number of trajectory points at 1kHz
    traj_length_ = static_cast<uint32_t>(traj_duration_ * 1000.0) + 1;
    traj_index_ = 0;
}

// ---------------------------------------------------------------------------
// move_to_station
// ---------------------------------------------------------------------------
bool TurntableController::move_to_station(Station target)
{
    TurntableState s = state_.load();
    if (s != TurntableState::IDLE && s != TurntableState::IN_POSITION) {
        fprintf(stderr, "[Turntable] Cannot move: state=%d\n",
                static_cast<int>(s));
        return false;
    }

    target_station_ = target;
    traj_start_  = drive_.actual_position();
    traj_target_ = station_counts(target);

    if (traj_start_ == traj_target_) {
        state_.store(TurntableState::IN_POSITION);
        return true;  // already there
    }

    // Begin unclamping sequence
    state_.store(TurntableState::UNCLAMPING);
    clamp_output_.store(false);
    unclamp_output_.store(true);
    clamp_timer_ = 0;

    printf("[Turntable] Move to station %d (from %d to %d counts)\n",
           static_cast<int>(target), traj_start_, traj_target_);
    return true;
}

// ---------------------------------------------------------------------------
// abort
// ---------------------------------------------------------------------------
void TurntableController::abort()
{
    drive_.quick_stop();
    state_.store(TurntableState::FAULT);
    clamp_output_.store(true);
    unclamp_output_.store(false);
}

// ---------------------------------------------------------------------------
// update  --  called every RT cycle
// ---------------------------------------------------------------------------
void TurntableController::update(uint8_t* domain_data,
                                 bool robots_clear,
                                 bool clamp_sensor_engaged,
                                 bool clamp_sensor_released)
{
    TurntableState s = state_.load();

    switch (s) {
        case TurntableState::IDLE:
        case TurntableState::IN_POSITION:
            // Nothing to do
            break;

        case TurntableState::UNCLAMPING:
            ++clamp_timer_;
            if (clamp_sensor_released) {
                // Clamp is open, wait for robot clearance
                state_.store(TurntableState::WAITING_CLEAR);
                clamp_timer_ = 0;
            } else if (clamp_timer_ > kClampTimeoutCycles) {
                fprintf(stderr, "[Turntable] Unclamp timeout\n");
                state_.store(TurntableState::FAULT);
            }
            break;

        case TurntableState::WAITING_CLEAR:
            if (robots_clear) {
                // Interlock satisfied -- begin motion
                generate_trajectory();
                drive_.set_operation_mode(OperationMode::CSP);
                state_.store(TurntableState::ACCELERATING);
            }
            break;

        case TurntableState::ACCELERATING:
        case TurntableState::CRUISE:
        case TurntableState::DECELERATING: {
            // Generate next position setpoint from S-curve
            double t = static_cast<double>(traj_index_) * 0.001;  // seconds
            double s_val = s_curve_profile(t, traj_duration_);

            double pos_deg = counts_to_deg(traj_start_)
                           + s_val * counts_to_deg(traj_target_ - traj_start_);
            int32_t pos_counts = deg_to_counts(pos_deg);

            drive_.set_target_position(pos_counts);
            ++traj_index_;

            // Update sub-state for diagnostics
            double progress = static_cast<double>(traj_index_)
                            / static_cast<double>(traj_length_);
            if (progress < 0.3) {
                state_.store(TurntableState::ACCELERATING);
            } else if (progress < 0.7) {
                state_.store(TurntableState::CRUISE);
            } else {
                state_.store(TurntableState::DECELERATING);
            }

            if (traj_index_ >= traj_length_) {
                // Trajectory complete, enter settling
                drive_.set_target_position(traj_target_);
                settle_counter_ = 0;
                state_.store(TurntableState::SETTLING);
            }
            break;
        }

        case TurntableState::SETTLING: {
            double err = position_error_deg();
            if (std::abs(err) <= in_pos_band_) {
                ++settle_counter_;
                if (settle_counter_ >= settling_cycles_) {
                    // Settled -- begin clamping
                    state_.store(TurntableState::CLAMPING);
                    clamp_output_.store(true);
                    unclamp_output_.store(false);
                    clamp_timer_ = 0;
                }
            } else {
                settle_counter_ = 0;
            }
            break;
        }

        case TurntableState::CLAMPING:
            ++clamp_timer_;
            if (clamp_sensor_engaged) {
                current_station_.store(target_station_);
                state_.store(TurntableState::IN_POSITION);
                printf("[Turntable] In position at station %d\n",
                       static_cast<int>(target_station_));
            } else if (clamp_timer_ > kClampTimeoutCycles) {
                fprintf(stderr, "[Turntable] Clamp timeout\n");
                state_.store(TurntableState::FAULT);
            }
            break;

        case TurntableState::FAULT:
            // Stay here until external reset
            break;
    }
}

// ---------------------------------------------------------------------------
// Status queries
// ---------------------------------------------------------------------------
bool TurntableController::in_position() const
{
    return state_.load() == TurntableState::IN_POSITION;
}

double TurntableController::position_error_deg() const
{
    int32_t target = station_counts(target_station_);
    int32_t actual = drive_.actual_position();
    return counts_to_deg(actual - target);
}
