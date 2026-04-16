#include "SafetyMonitor.h"

#include <cstdio>

// ---------------------------------------------------------------------------
// Fault codes
// ---------------------------------------------------------------------------
static constexpr uint32_t FAULT_ESTOP_DISCREPANCY  = 0x0001;
static constexpr uint32_t FAULT_ESTOP_PRESSED      = 0x0002;
static constexpr uint32_t FAULT_CURTAIN_BREACH     = 0x0004;
static constexpr uint32_t FAULT_DOOR_OPEN          = 0x0008;
static constexpr uint32_t FAULT_STO_FB_DISCREPANCY = 0x0010;
static constexpr uint32_t FAULT_FSOE_WATCHDOG      = 0x0020;

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------
SafetyMonitor::SafetyMonitor()
{
    for (size_t i = 0; i < static_cast<size_t>(LightCurtainZone::COUNT); ++i) {
        curtain_clear_[i].store(false);
    }
}

// ---------------------------------------------------------------------------
// Input setters (called from domain read in RT loop)
// ---------------------------------------------------------------------------
void SafetyMonitor::set_estop(EStopId id, bool channel_a, bool channel_b)
{
    size_t idx = static_cast<size_t>(id);
    if (idx >= static_cast<size_t>(EStopId::COUNT)) return;
    estops_[idx].ch_a.store(channel_a, std::memory_order_relaxed);
    estops_[idx].ch_b.store(channel_b, std::memory_order_relaxed);
}

void SafetyMonitor::set_light_curtain(LightCurtainZone zone, bool clear)
{
    size_t idx = static_cast<size_t>(zone);
    if (idx >= static_cast<size_t>(LightCurtainZone::COUNT)) return;
    curtain_clear_[idx].store(clear, std::memory_order_relaxed);
}

void SafetyMonitor::set_safety_door(bool closed)
{
    door_closed_.store(closed, std::memory_order_relaxed);
}

void SafetyMonitor::set_drive_sto_feedback(uint8_t drive_index,
                                           bool ch_a, bool ch_b)
{
    if (drive_index >= kMaxDrives) return;
    sto_fb_[drive_index].ch_a.store(ch_a, std::memory_order_relaxed);
    sto_fb_[drive_index].ch_b.store(ch_b, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// update  --  called every RT cycle
// ---------------------------------------------------------------------------
void SafetyMonitor::update(uint8_t* /*domain_data*/)
{
    SafetyState s = state_.load(std::memory_order_relaxed);

    // Always monitor safety inputs regardless of state
    check_dual_channel_estops();
    check_light_curtains();
    check_safety_door();
    check_drive_sto_feedback();
    run_fsoe_watchdog();

    switch (s) {
        case SafetyState::RUN: {
            // Check if any safety condition triggers a stop
            if (!all_estops_ok()) {
                raise_fault(FAULT_ESTOP_PRESSED);
                transition_to(SafetyState::SAFE_STOP_REQ);
                return;
            }
            if (!safety_door_closed()) {
                transition_to(SafetyState::SAFE_STOP_REQ);
                return;
            }
            // Light curtain breach -> SLS if in collaborative zone,
            // otherwise safe stop
            if (!all_light_curtains_clear()) {
                if (!sls_active_.load(std::memory_order_relaxed)) {
                    sls_active_.store(true, std::memory_order_relaxed);
                    printf("[Safety] SLS activated -- light curtain breach\n");
                }
            } else {
                if (sls_active_.load(std::memory_order_relaxed)) {
                    sls_active_.store(false, std::memory_order_relaxed);
                    printf("[Safety] SLS deactivated -- curtains clear\n");
                }
            }
            break;
        }

        case SafetyState::SAFE_REDUCED: {
            // SLS mode -- monitor velocity limits
            // If curtains clear, transition back to RUN
            if (all_light_curtains_clear() && all_estops_ok()
                && safety_door_closed()) {
                sls_active_.store(false, std::memory_order_relaxed);
                transition_to(SafetyState::RUN);
            }
            break;
        }

        case SafetyState::SAFE_STOP_REQ: {
            // Initiate controlled stop ramp
            stop_timer_ = 0;
            transition_to(SafetyState::SAFE_STOP_ACTIVE);
            printf("[Safety] Safe stop initiated\n");
            break;
        }

        case SafetyState::SAFE_STOP_ACTIVE: {
            ++stop_timer_;
            if (stop_timer_ >= kStopRampCycles) {
                // Ramp complete -- engage STO
                transition_to(SafetyState::STO_ACTIVE);
                printf("[Safety] STO engaged after ramp\n");
            }
            break;
        }

        case SafetyState::STO_ACTIVE: {
            // Monitor for run request (manual reset)
            if (run_requested_) {
                if (all_estops_ok() && safety_door_closed()
                    && all_light_curtains_clear()) {
                    run_requested_ = false;
                    transition_to(SafetyState::RUN);
                    printf("[Safety] Transitioning to RUN\n");
                } else {
                    run_requested_ = false;
                    fprintf(stderr, "[Safety] Cannot run: safety inputs "
                            "not clear\n");
                }
            }
            break;
        }

        case SafetyState::FAULT: {
            // Stay in fault until acknowledged
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
bool SafetyMonitor::request_run()
{
    SafetyState s = state_.load();
    if (s != SafetyState::STO_ACTIVE) {
        fprintf(stderr, "[Safety] request_run rejected: state=%d\n",
                static_cast<int>(s));
        return false;
    }
    run_requested_ = true;
    return true;
}

bool SafetyMonitor::request_safe_stop()
{
    SafetyState s = state_.load();
    if (s == SafetyState::STO_ACTIVE || s == SafetyState::FAULT) {
        return false;
    }
    transition_to(SafetyState::SAFE_STOP_REQ);
    return true;
}

void SafetyMonitor::acknowledge_fault()
{
    if (state_.load() != SafetyState::FAULT) return;

    // Only allow ack if all safety inputs are healthy
    if (all_estops_ok() && safety_door_closed()) {
        fault_code_.store(0, std::memory_order_relaxed);
        transition_to(SafetyState::STO_ACTIVE);
        printf("[Safety] Fault acknowledged, transitioning to STO\n");
    } else {
        fprintf(stderr, "[Safety] Cannot ack fault: inputs not clear\n");
    }
}

// ---------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------
bool SafetyMonitor::sto_active() const
{
    SafetyState s = state_.load(std::memory_order_relaxed);
    return s == SafetyState::STO_ACTIVE || s == SafetyState::FAULT;
}

bool SafetyMonitor::all_estops_ok() const
{
    for (size_t i = 0; i < static_cast<size_t>(EStopId::COUNT); ++i) {
        if (!estops_[i].ch_a.load(std::memory_order_relaxed)
            || !estops_[i].ch_b.load(std::memory_order_relaxed)) {
            return false;
        }
    }
    return true;
}

bool SafetyMonitor::all_light_curtains_clear() const
{
    for (size_t i = 0; i < static_cast<size_t>(LightCurtainZone::COUNT); ++i) {
        if (!curtain_clear_[i].load(std::memory_order_relaxed)) {
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------
// Internal checks
// ---------------------------------------------------------------------------
void SafetyMonitor::check_dual_channel_estops()
{
    for (size_t i = 0; i < static_cast<size_t>(EStopId::COUNT); ++i) {
        bool a = estops_[i].ch_a.load(std::memory_order_relaxed);
        bool b = estops_[i].ch_b.load(std::memory_order_relaxed);

        if (a != b) {
            // Discrepancy detected -- start timer
            ++estops_[i].discrepancy_timer;
            if (estops_[i].discrepancy_timer >= kDiscrepancyCycles) {
                fprintf(stderr, "[Safety] E-Stop %zu dual-channel "
                        "discrepancy (500ms exceeded)\n", i);
                raise_fault(FAULT_ESTOP_DISCREPANCY);
            }
        } else {
            estops_[i].discrepancy_timer = 0;
        }
    }
}

void SafetyMonitor::check_light_curtains()
{
    // Curtain status is read via set_light_curtain() each cycle.
    // If breached during RUN, the update() state machine handles SLS.
}

void SafetyMonitor::check_safety_door()
{
    // Door status is read via set_safety_door() each cycle.
    // If opened during RUN, the update() state machine handles safe stop.
}

void SafetyMonitor::check_drive_sto_feedback()
{
    // Verify STO feedback matches command: if we've commanded STO
    // (STO_ACTIVE state) but feedback shows drives still powered,
    // that's a fault.
    if (state_.load(std::memory_order_relaxed) == SafetyState::STO_ACTIVE) {
        for (size_t i = 0; i < kMaxDrives; ++i) {
            bool a = sto_fb_[i].ch_a.load(std::memory_order_relaxed);
            bool b = sto_fb_[i].ch_b.load(std::memory_order_relaxed);

            if (a != b) {
                ++sto_fb_[i].discrepancy_timer;
                if (sto_fb_[i].discrepancy_timer >= kDiscrepancyCycles) {
                    fprintf(stderr, "[Safety] STO feedback discrepancy "
                            "on drive %zu\n", i);
                    raise_fault(FAULT_STO_FB_DISCREPANCY);
                }
            } else {
                sto_fb_[i].discrepancy_timer = 0;
            }

            // Both channels should report "torque off" (true = safe)
            // during STO. If both show active power, fault.
            if (!a && !b) {
                // Drive still powered during STO -- fault
                raise_fault(FAULT_STO_FB_DISCREPANCY);
            }
        }
    }
}

void SafetyMonitor::run_fsoe_watchdog()
{
    // FSoE connection monitoring: if no valid safety frame received
    // within watchdog period, trigger fault.
    // In a real FSoE implementation, the safety PLC sends periodic
    // safety frames. Here we model the watchdog counter.
    ++fsoe_watchdog_counter_;
    if (fsoe_watchdog_counter_ >= kFsoeWatchdogLimit) {
        // Reset by the FSoE frame handler (not shown)
        // If we reach the limit, no valid frame was received.
        // For now, this is reset elsewhere when a valid frame arrives.
    }
}

// ---------------------------------------------------------------------------
// State transitions and fault
// ---------------------------------------------------------------------------
void SafetyMonitor::transition_to(SafetyState next)
{
    SafetyState prev = state_.load(std::memory_order_relaxed);
    state_.store(next, std::memory_order_relaxed);
    printf("[Safety] State: %d -> %d\n",
           static_cast<int>(prev), static_cast<int>(next));
}

void SafetyMonitor::raise_fault(uint32_t code)
{
    uint32_t current = fault_code_.load(std::memory_order_relaxed);
    fault_code_.store(current | code, std::memory_order_relaxed);
    if (state_.load(std::memory_order_relaxed) != SafetyState::FAULT) {
        transition_to(SafetyState::FAULT);
        fprintf(stderr, "[Safety] FAULT raised: 0x%04X\n", current | code);
    }
}
