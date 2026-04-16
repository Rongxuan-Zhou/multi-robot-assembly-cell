#pragma once

#include <atomic>
#include <cstdint>
#include <string>

class RobotSlaveDriver;

// ---------------------------------------------------------------------------
// Screw specification
// ---------------------------------------------------------------------------
struct ScrewSpec {
    std::string id;             // e.g. "M2.5x4"
    double target_torque_nm;    // e.g. 0.3
    double torque_tolerance;    // +/- fraction (e.g. 0.10 = 10%)
    double target_angle_deg;    // expected total angle, 0 = don't check
    double angle_tolerance_deg; // +/- degrees
    double approach_speed_rpm;
    double snug_speed_rpm;
    double final_speed_rpm;
    double snug_torque_nm;      // transition threshold from approach to snug
};

// ---------------------------------------------------------------------------
// Tightening result per screw
// ---------------------------------------------------------------------------
struct ScrewResult {
    bool   ok          = false;
    double final_torque = 0.0;
    double final_angle  = 0.0;
    double duration_ms  = 0.0;
    uint32_t screw_index = 0;
    uint64_t timestamp_ns = 0;
};

enum class ScrewPhase : uint8_t {
    IDLE,
    FEEDER_ADVANCE,   // advance screw feeder to pick position
    APPROACH,         // fast run-down to contact
    SNUG,             // slow to snug torque
    FINAL_TORQUE,     // controlled final tightening
    VERIFY,           // hold and verify torque
    RETRACT,          // retract driver bit
    COMPLETE,
    FAULT,
};

// ---------------------------------------------------------------------------
// ScrewDriverController  --  screw tightening sequence
// ---------------------------------------------------------------------------
class ScrewDriverController {
public:
    ScrewDriverController(RobotSlaveDriver& spindle_drive,
                          const std::string& name);
    ~ScrewDriverController() = default;

    // ----- Commands -----
    bool start_tightening(const ScrewSpec& spec, uint32_t screw_index);
    void abort();
    bool start_bit_change(uint8_t target_bit_id);

    // ----- Cyclic update (called from RT thread) -----
    void update(uint8_t* domain_data);

    // ----- Status -----
    ScrewPhase        phase()       const { return phase_.load(); }
    const ScrewResult& last_result() const { return last_result_; }
    bool              is_busy()     const;
    uint8_t           current_bit() const { return current_bit_; }

    // ----- Feeder -----
    void set_feeder_bit_count(uint8_t n) { feeder_bit_count_ = n; }
    uint8_t feeder_position() const { return feeder_position_; }

private:
    void run_tightening_fsm(uint8_t* domain_data);
    void run_bit_change_fsm();
    bool check_torque_ok() const;
    bool check_angle_ok() const;

    RobotSlaveDriver& spindle_;
    std::string name_;

    // Current spec
    ScrewSpec   active_spec_{};
    uint32_t    active_screw_index_ = 0;

    // Phase tracking
    std::atomic<ScrewPhase> phase_{ScrewPhase::IDLE};
    uint32_t phase_timer_ = 0;

    // Measurement accumulators (updated in RT loop, no allocation)
    double accumulated_angle_ = 0.0;
    double peak_torque_       = 0.0;
    int32_t angle_ref_pos_    = 0;
    uint64_t start_time_ns_   = 0;

    // Result (written at end of sequence, read outside RT)
    ScrewResult last_result_{};

    // Feeder
    uint8_t feeder_bit_count_ = 30;
    uint8_t feeder_position_  = 0;

    // Bit changer
    uint8_t current_bit_      = 0;
    uint8_t target_bit_       = 0;
    bool    bit_change_active_= false;

    // Verification hold
    static constexpr uint32_t kVerifyHoldCycles = 200;   // 200ms at 1kHz
    static constexpr double   kTorqueDecayLimit = 0.05;  // max 5% drop
};
