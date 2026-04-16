#pragma once

#include <atomic>
#include <cstdint>

class RobotSlaveDriver;

// ---------------------------------------------------------------------------
// Turntable stations (120-degree indexed positions)
// ---------------------------------------------------------------------------
enum class Station : uint8_t {
    STATION_1 = 0,   //   0 deg  -- loading
    STATION_2 = 1,   // 120 deg  -- assembly
    STATION_3 = 2,   // 240 deg  -- unloading
};

enum class TurntableState : uint8_t {
    IDLE,
    UNCLAMPING,
    WAITING_CLEAR,      // waiting for robot interlock
    ACCELERATING,
    CRUISE,
    DECELERATING,
    SETTLING,
    CLAMPING,
    IN_POSITION,
    FAULT,
};

// ---------------------------------------------------------------------------
// TurntableController  --  120-degree servo-driven indexing table
// ---------------------------------------------------------------------------
class TurntableController {
public:
    // encoder_counts_per_rev: total counts for 360 degrees
    explicit TurntableController(RobotSlaveDriver& drive,
                                 int32_t encoder_counts_per_rev = 1'048'576);
    ~TurntableController() = default;

    // ----- Motion commands -----
    bool move_to_station(Station target);
    void abort();

    // ----- Cyclic update (called from RT thread) -----
    void update(uint8_t* domain_data,
                bool robots_clear,          // interlock input
                bool clamp_sensor_engaged,  // clamp proximity sensor
                bool clamp_sensor_released);

    // ----- Status -----
    TurntableState state()            const { return state_.load(); }
    Station        current_station()  const { return current_station_.load(); }
    bool           in_position()      const;
    double         position_error_deg() const;

    // ----- I/O -----
    bool clamp_output()   const { return clamp_output_.load(); }
    bool unclamp_output() const { return unclamp_output_.load(); }

    // ----- Tuning -----
    void set_max_velocity(double deg_per_sec)    { max_vel_ = deg_per_sec; }
    void set_max_acceleration(double deg_per_s2) { max_acc_ = deg_per_s2; }
    void set_in_position_band(double deg)        { in_pos_band_ = deg; }
    void set_settling_cycles(uint32_t n)         { settling_cycles_ = n; }

private:
    int32_t deg_to_counts(double deg) const;
    double  counts_to_deg(int32_t c)  const;
    int32_t station_counts(Station s) const;
    double  s_curve_profile(double t, double T) const;
    void    generate_trajectory();

    RobotSlaveDriver& drive_;
    int32_t encoder_counts_per_rev_;

    // Motion profile parameters
    double max_vel_       = 60.0;    // deg/s
    double max_acc_       = 120.0;   // deg/s^2
    double in_pos_band_   = 0.01;    // deg
    uint32_t settling_cycles_ = 50;  // cycles within band before "settled"

    // Trajectory state
    int32_t  traj_start_    = 0;
    int32_t  traj_target_   = 0;
    uint32_t traj_length_   = 0;     // total trajectory points
    uint32_t traj_index_    = 0;
    double   traj_duration_ = 0.0;   // seconds

    // Settling counter
    uint32_t settle_counter_ = 0;

    // Pneumatic clamp outputs
    std::atomic<bool> clamp_output_{true};
    std::atomic<bool> unclamp_output_{false};
    uint32_t clamp_timer_ = 0;
    static constexpr uint32_t kClampTimeoutCycles = 500;  // 0.5s at 1kHz

    // State
    std::atomic<TurntableState> state_{TurntableState::IDLE};
    std::atomic<Station>        current_station_{Station::STATION_1};
    Station                     target_station_{Station::STATION_1};
};
