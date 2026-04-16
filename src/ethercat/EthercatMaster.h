#pragma once

#include <ecrt.h>

#include <atomic>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>
#include <vector>

// ---------------------------------------------------------------------------
// Forward declarations
// ---------------------------------------------------------------------------
class RobotSlaveDriver;
class TurntableController;
class ScrewDriverController;
class SafetyMonitor;

// ---------------------------------------------------------------------------
// Enumerations
// ---------------------------------------------------------------------------
enum class SlaveState : uint8_t {
    INIT    = 0x01,
    PREOP   = 0x02,
    SAFEOP  = 0x04,
    OP      = 0x08,
    UNKNOWN = 0x00
};

// ---------------------------------------------------------------------------
// Jitter statistics (lock-free, updated from the RT thread)
// ---------------------------------------------------------------------------
struct CycleStats {
    std::atomic<int64_t> min_jitter_ns{0};
    std::atomic<int64_t> max_jitter_ns{0};
    std::atomic<int64_t> avg_jitter_ns{0};
    std::atomic<uint64_t> cycle_count{0};
    std::atomic<uint64_t> overrun_count{0};
};

// ---------------------------------------------------------------------------
// Slave descriptor used during configuration
// ---------------------------------------------------------------------------
struct SlaveDescriptor {
    uint16_t alias;
    uint16_t position;
    uint32_t vendor_id;
    uint32_t product_code;
    std::string name;
};

// ---------------------------------------------------------------------------
// EthercatMaster  --  IgH EtherCAT Master wrapper
// ---------------------------------------------------------------------------
class EthercatMaster {
public:
    explicit EthercatMaster(unsigned master_index = 0,
                            uint32_t cycle_time_us = 1000);
    ~EthercatMaster();

    // Non-copyable / non-movable (owns kernel resources)
    EthercatMaster(const EthercatMaster&)            = delete;
    EthercatMaster& operator=(const EthercatMaster&) = delete;

    // ----- Life-cycle -----
    bool initialize();
    bool configure_slaves();
    bool activate();
    void start_cyclic_task(int cpu_core = -1);   // -1 = no affinity
    void request_stop();
    void wait_for_stop();

    // ----- SDO access (non-RT, called from management thread) -----
    int sdo_read(uint16_t slave_pos, uint16_t index, uint8_t subindex,
                 uint8_t* data, size_t size);
    int sdo_write(uint16_t slave_pos, uint16_t index, uint8_t subindex,
                  const uint8_t* data, size_t size);

    // ----- Runtime queries -----
    SlaveState       master_state() const;
    const CycleStats& cycle_stats() const { return stats_; }

    // ----- Domain data accessors (used by slave drivers) -----
    uint8_t* domain_data() const { return domain_data_; }

    // ----- Emergency callback registration -----
    using EmergencyCallback = std::function<void(uint16_t slave_pos,
                                                  uint16_t error_code,
                                                  uint8_t  error_register,
                                                  const uint8_t* mfr_data)>;
    void set_emergency_callback(EmergencyCallback cb);

private:
    void cyclic_task();                          // RT loop body
    void check_master_state();
    void check_slave_states();
    void handle_emergency_messages();

    // IgH handles
    ec_master_t*        master_       = nullptr;
    ec_domain_t*        domain_       = nullptr;
    ec_master_state_t   master_st_{};
    uint8_t*            domain_data_  = nullptr;

    // Slave configurations (populated during configure_slaves)
    std::vector<ec_slave_config_t*> slave_cfgs_;
    std::vector<SlaveDescriptor>    slave_descs_;

    // Timing
    uint32_t cycle_time_us_;
    unsigned master_index_;

    // RT thread control
    std::atomic<bool> running_{false};
    pthread_t         rt_thread_{};

    // Statistics
    CycleStats stats_;

    // Emergency
    EmergencyCallback emg_cb_;
    std::mutex        emg_mutex_;
};
