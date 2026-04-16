#pragma once

#include <atomic>
#include <cstdint>

// ---------------------------------------------------------------------------
// Safety states
// ---------------------------------------------------------------------------
enum class SafetyState : uint8_t {
    RUN,                // normal operation
    SAFE_REDUCED,       // SLS active (collaborative zone)
    SAFE_STOP_REQ,      // controlled stop initiated
    SAFE_STOP_ACTIVE,   // drives decelerating
    STO_ACTIVE,         // Safe Torque Off engaged
    FAULT,              // safety fault -- manual reset required
};

// ---------------------------------------------------------------------------
// E-Stop / zone identifiers
// ---------------------------------------------------------------------------
enum class EStopId : uint8_t {
    PANEL_FRONT = 0,
    PANEL_REAR  = 1,
    PENDANT     = 2,
    COUNT       = 3,
};

enum class LightCurtainZone : uint8_t {
    LOADING    = 0,
    UNLOADING  = 1,
    COUNT      = 2,
};

// ---------------------------------------------------------------------------
// SafetyMonitor  --  FSoE protocol handler + safety logic
// ---------------------------------------------------------------------------
class SafetyMonitor {
public:
    SafetyMonitor();
    ~SafetyMonitor() = default;

    // ----- Cyclic update (called from RT thread) -----
    void update(uint8_t* domain_data);

    // ----- Input setters (called from domain read in RT loop) -----
    void set_estop(EStopId id, bool channel_a, bool channel_b);
    void set_light_curtain(LightCurtainZone zone, bool clear);
    void set_safety_door(bool closed);
    void set_drive_sto_feedback(uint8_t drive_index, bool ch_a, bool ch_b);

    // ----- Commands -----
    bool request_run();            // transition STO -> RUN
    bool request_safe_stop();      // controlled stop -> STO
    void acknowledge_fault();

    // ----- Queries -----
    SafetyState state()                    const { return state_.load(); }
    bool        sto_active()               const;
    bool        sls_active()               const { return sls_active_.load(); }
    bool        all_estops_ok()            const;
    bool        all_light_curtains_clear() const;
    bool        safety_door_closed()       const { return door_closed_.load(); }
    uint32_t    fault_code()               const { return fault_code_.load(); }

    // ----- SLS configuration -----
    void set_sls_velocity_limit(double limit_deg_per_s);
    double sls_velocity_limit() const { return sls_vel_limit_; }

private:
    void run_fsoe_watchdog();
    void check_dual_channel_estops();
    void check_light_curtains();
    void check_safety_door();
    void check_drive_sto_feedback();
    void transition_to(SafetyState next);
    void raise_fault(uint32_t code);

    std::atomic<SafetyState> state_{SafetyState::STO_ACTIVE};

    // E-Stop dual-channel inputs
    struct DualChannel {
        std::atomic<bool> ch_a{false};
        std::atomic<bool> ch_b{false};
        uint32_t discrepancy_timer{0};
    };
    DualChannel estops_[static_cast<size_t>(EStopId::COUNT)];

    // Light curtains
    std::atomic<bool> curtain_clear_[static_cast<size_t>(LightCurtainZone::COUNT)]{};

    // Safety door
    std::atomic<bool> door_closed_{false};

    // STO feedback per drive (dual-channel)
    static constexpr size_t kMaxDrives = 8;
    DualChannel sto_fb_[kMaxDrives];

    // SLS
    std::atomic<bool> sls_active_{false};
    double sls_vel_limit_ = 250.0;   // deg/s

    // Discrepancy timing
    static constexpr uint32_t kDiscrepancyCycles = 500;  // 500ms at 1kHz

    // Stop ramp
    uint32_t stop_timer_     = 0;
    static constexpr uint32_t kStopRampCycles = 1000;  // 1s ramp

    // Fault
    std::atomic<uint32_t> fault_code_{0};
    bool run_requested_ = false;

    // FSoE watchdog
    uint32_t fsoe_watchdog_counter_ = 0;
    static constexpr uint32_t kFsoeWatchdogLimit = 100;  // 100ms
};
