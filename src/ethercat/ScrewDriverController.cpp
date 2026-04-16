#include "ScrewDriverController.h"
#include "RobotSlaveDriver.h"

#include <cmath>
#include <cstdio>
#include <ctime>

// ---------------------------------------------------------------------------
// Helper: monotonic timestamp in nanoseconds (no allocation)
// ---------------------------------------------------------------------------
static uint64_t now_ns()
{
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1'000'000'000ULL
         + static_cast<uint64_t>(ts.tv_nsec);
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------
ScrewDriverController::ScrewDriverController(RobotSlaveDriver& spindle_drive,
                                             const std::string& name)
    : spindle_(spindle_drive)
    , name_(name)
{
}

// ---------------------------------------------------------------------------
// start_tightening
// ---------------------------------------------------------------------------
bool ScrewDriverController::start_tightening(const ScrewSpec& spec,
                                             uint32_t screw_index)
{
    ScrewPhase p = phase_.load();
    if (p != ScrewPhase::IDLE && p != ScrewPhase::COMPLETE) {
        fprintf(stderr, "[%s] Cannot start: phase=%d\n",
                name_.c_str(), static_cast<int>(p));
        return false;
    }

    if (spindle_.current_drive_state() != DriveState::OPERATION_ENABLED) {
        fprintf(stderr, "[%s] Spindle not enabled\n", name_.c_str());
        return false;
    }

    active_spec_        = spec;
    active_screw_index_ = screw_index;
    accumulated_angle_  = 0.0;
    peak_torque_        = 0.0;
    angle_ref_pos_      = spindle_.actual_position();
    start_time_ns_      = now_ns();
    phase_timer_        = 0;

    // Advance feeder
    phase_.store(ScrewPhase::FEEDER_ADVANCE);
    return true;
}

// ---------------------------------------------------------------------------
// abort
// ---------------------------------------------------------------------------
void ScrewDriverController::abort()
{
    spindle_.set_target_velocity(0);
    spindle_.set_target_torque(0);
    phase_.store(ScrewPhase::IDLE);
    phase_timer_ = 0;
}

// ---------------------------------------------------------------------------
// start_bit_change
// ---------------------------------------------------------------------------
bool ScrewDriverController::start_bit_change(uint8_t target_bit_id)
{
    ScrewPhase p = phase_.load();
    if (p != ScrewPhase::IDLE && p != ScrewPhase::COMPLETE) {
        return false;
    }
    target_bit_ = target_bit_id;
    bit_change_active_ = true;
    return true;
}

// ---------------------------------------------------------------------------
// is_busy
// ---------------------------------------------------------------------------
bool ScrewDriverController::is_busy() const
{
    ScrewPhase p = phase_.load();
    return p != ScrewPhase::IDLE && p != ScrewPhase::COMPLETE
        && p != ScrewPhase::FAULT;
}

// ---------------------------------------------------------------------------
// update  --  called from RT thread every cycle
// ---------------------------------------------------------------------------
void ScrewDriverController::update(uint8_t* domain_data)
{
    if (bit_change_active_) {
        run_bit_change_fsm();
        return;
    }

    ScrewPhase p = phase_.load();
    if (p == ScrewPhase::IDLE || p == ScrewPhase::COMPLETE
        || p == ScrewPhase::FAULT) {
        return;
    }

    run_tightening_fsm(domain_data);
}

// ---------------------------------------------------------------------------
// run_tightening_fsm
// ---------------------------------------------------------------------------
void ScrewDriverController::run_tightening_fsm(uint8_t* domain_data)
{
    // Read current torque (actual_torque is in 0.1% of rated torque)
    int16_t raw_torque = spindle_.actual_torque();
    // Convert to Nm: assume 1.0 Nm rated torque for the spindle
    double current_torque_nm = static_cast<double>(raw_torque) / 1000.0;
    if (current_torque_nm > peak_torque_) {
        peak_torque_ = current_torque_nm;
    }

    // Update accumulated angle
    int32_t current_pos = spindle_.actual_position();
    // Assume 4096 counts/rev for spindle encoder
    static constexpr double kCountsPerDeg = 4096.0 / 360.0;
    accumulated_angle_ = static_cast<double>(current_pos - angle_ref_pos_)
                       / kCountsPerDeg;

    ++phase_timer_;

    ScrewPhase p = phase_.load();

    switch (p) {
        case ScrewPhase::FEEDER_ADVANCE: {
            // Advance feeder to next screw position
            // In a real system this would pulse a digital output
            feeder_position_ = (feeder_position_ + 1) % feeder_bit_count_;
            // Allow 100ms for feeder to settle
            if (phase_timer_ >= 100) {
                phase_timer_ = 0;
                phase_.store(ScrewPhase::APPROACH);
                spindle_.set_operation_mode(OperationMode::CSV);
                // Approach speed in counts/s
                int32_t approach_vel = static_cast<int32_t>(
                    active_spec_.approach_speed_rpm / 60.0 * 4096.0);
                spindle_.set_target_velocity(approach_vel);
            }
            break;
        }

        case ScrewPhase::APPROACH: {
            // Fast run-down until snug torque is detected
            if (current_torque_nm >= active_spec_.snug_torque_nm) {
                // Transition to snug phase
                phase_timer_ = 0;
                angle_ref_pos_ = current_pos;  // reset angle measurement
                accumulated_angle_ = 0.0;
                phase_.store(ScrewPhase::SNUG);

                int32_t snug_vel = static_cast<int32_t>(
                    active_spec_.snug_speed_rpm / 60.0 * 4096.0);
                spindle_.set_target_velocity(snug_vel);
                printf("[%s] Snug detected at %.3f Nm\n",
                       name_.c_str(), current_torque_nm);
            }

            // Timeout: 5 seconds of approach
            if (phase_timer_ > 5000) {
                fprintf(stderr, "[%s] Approach timeout\n", name_.c_str());
                phase_.store(ScrewPhase::FAULT);
                spindle_.set_target_velocity(0);
            }
            break;
        }

        case ScrewPhase::SNUG: {
            // Controlled tightening toward target torque
            // Switch to torque mode near target
            if (current_torque_nm >= active_spec_.target_torque_nm * 0.8) {
                phase_timer_ = 0;
                phase_.store(ScrewPhase::FINAL_TORQUE);
                spindle_.set_operation_mode(OperationMode::CST);

                int16_t target_trq = static_cast<int16_t>(
                    active_spec_.target_torque_nm * 1000.0);  // permille
                spindle_.set_target_torque(target_trq);

                int32_t final_vel = static_cast<int32_t>(
                    active_spec_.final_speed_rpm / 60.0 * 4096.0);
                spindle_.set_target_velocity(final_vel);
            }
            break;
        }

        case ScrewPhase::FINAL_TORQUE: {
            // Hold at target torque until stable
            bool torque_reached = current_torque_nm
                                >= active_spec_.target_torque_nm * 0.95;
            if (torque_reached) {
                phase_timer_ = 0;
                peak_torque_ = current_torque_nm;
                phase_.store(ScrewPhase::VERIFY);
                // Stop rotation, hold torque
                spindle_.set_target_velocity(0);
            }

            // Timeout: 3 seconds
            if (phase_timer_ > 3000) {
                fprintf(stderr, "[%s] Final torque timeout at %.3f / %.3f Nm\n",
                        name_.c_str(), current_torque_nm,
                        active_spec_.target_torque_nm);
                phase_.store(ScrewPhase::FAULT);
                spindle_.set_target_velocity(0);
                spindle_.set_target_torque(0);
            }
            break;
        }

        case ScrewPhase::VERIFY: {
            // Hold for kVerifyHoldCycles and check torque doesn't decay
            if (phase_timer_ >= kVerifyHoldCycles) {
                // Check final torque is still within spec
                bool ok = check_torque_ok() && check_angle_ok();

                last_result_.ok           = ok;
                last_result_.final_torque  = current_torque_nm;
                last_result_.final_angle   = accumulated_angle_;
                last_result_.screw_index   = active_screw_index_;
                last_result_.timestamp_ns  = now_ns();
                last_result_.duration_ms   = static_cast<double>(
                    last_result_.timestamp_ns - start_time_ns_) / 1'000'000.0;

                printf("[%s] Screw %u: %s | torque=%.3f Nm | angle=%.1f deg "
                       "| %.0f ms\n",
                       name_.c_str(), active_screw_index_,
                       ok ? "OK" : "NG",
                       current_torque_nm, accumulated_angle_,
                       last_result_.duration_ms);

                phase_timer_ = 0;
                phase_.store(ScrewPhase::RETRACT);
                spindle_.set_target_torque(0);
                // Reverse slowly to retract
                spindle_.set_operation_mode(OperationMode::CSV);
                spindle_.set_target_velocity(-500);
            }

            // Check torque decay during hold
            if (current_torque_nm < peak_torque_ * (1.0 - kTorqueDecayLimit)) {
                fprintf(stderr, "[%s] Torque decay detected during verify\n",
                        name_.c_str());
                last_result_.ok = false;
                last_result_.final_torque = current_torque_nm;
                last_result_.final_angle  = accumulated_angle_;
                last_result_.screw_index  = active_screw_index_;
                last_result_.timestamp_ns = now_ns();
                last_result_.duration_ms  = static_cast<double>(
                    last_result_.timestamp_ns - start_time_ns_) / 1'000'000.0;
                phase_.store(ScrewPhase::RETRACT);
                spindle_.set_target_torque(0);
                spindle_.set_operation_mode(OperationMode::CSV);
                spindle_.set_target_velocity(-500);
            }
            break;
        }

        case ScrewPhase::RETRACT: {
            // Retract for 200ms then stop
            if (phase_timer_ >= 200) {
                spindle_.set_target_velocity(0);
                phase_.store(ScrewPhase::COMPLETE);
            }
            break;
        }

        default:
            break;
    }
}

// ---------------------------------------------------------------------------
// run_bit_change_fsm  --  simplified bit changer sequence
// ---------------------------------------------------------------------------
void ScrewDriverController::run_bit_change_fsm()
{
    // In a real system this would coordinate pneumatic actuators
    // and verify bit presence via digital I/O.
    // Simplified: set current_bit_ after a delay.
    ++phase_timer_;
    if (phase_timer_ >= 500) {   // 500ms for bit change
        current_bit_ = target_bit_;
        bit_change_active_ = false;
        phase_timer_ = 0;
        printf("[%s] Bit changed to %u\n", name_.c_str(), current_bit_);
    }
}

// ---------------------------------------------------------------------------
// Torque / angle verification
// ---------------------------------------------------------------------------
bool ScrewDriverController::check_torque_ok() const
{
    double low  = active_spec_.target_torque_nm
                * (1.0 - active_spec_.torque_tolerance);
    double high = active_spec_.target_torque_nm
                * (1.0 + active_spec_.torque_tolerance);
    return (peak_torque_ >= low) && (peak_torque_ <= high);
}

bool ScrewDriverController::check_angle_ok() const
{
    if (active_spec_.target_angle_deg <= 0.0) {
        return true;  // angle check disabled
    }
    double low  = active_spec_.target_angle_deg
                - active_spec_.angle_tolerance_deg;
    double high = active_spec_.target_angle_deg
                + active_spec_.angle_tolerance_deg;
    return (accumulated_angle_ >= low) && (accumulated_angle_ <= high);
}
