#pragma once

#include <ecrt.h>

#include <atomic>
#include <cstdint>
#include <string>

// ---------------------------------------------------------------------------
// CiA 402 operation modes
// ---------------------------------------------------------------------------
enum class OperationMode : int8_t {
    NO_MODE   =  0,
    PP        =  1,   // Profile Position
    PV        =  3,   // Profile Velocity
    PT        =  4,   // Profile Torque
    HOMING    =  6,
    IP        =  7,   // Interpolated Position
    CSP       =  8,   // Cyclic Synchronous Position
    CSV       =  9,   // Cyclic Synchronous Velocity
    CST       = 10,   // Cyclic Synchronous Torque
};

// ---------------------------------------------------------------------------
// CiA 402 state machine states  (derived from statusword)
// ---------------------------------------------------------------------------
enum class DriveState : uint8_t {
    NOT_READY_TO_SWITCH_ON,
    SWITCH_ON_DISABLED,
    READY_TO_SWITCH_ON,
    SWITCHED_ON,
    OPERATION_ENABLED,
    QUICK_STOP_ACTIVE,
    FAULT_REACTION_ACTIVE,
    FAULT,
    UNKNOWN
};

// ---------------------------------------------------------------------------
// PDO offsets inside the EtherCAT domain
// ---------------------------------------------------------------------------
struct RobotPdoOffsets {
    // RxPDO (master -> slave)
    unsigned int controlword;
    unsigned int target_position;
    unsigned int target_velocity;
    unsigned int target_torque;
    unsigned int mode_of_operation;
    unsigned int digital_outputs;

    // TxPDO (slave -> master)
    unsigned int statusword;
    unsigned int actual_position;
    unsigned int actual_velocity;
    unsigned int actual_torque;
    unsigned int mode_of_operation_display;
    unsigned int digital_inputs;
};

// ---------------------------------------------------------------------------
// RobotSlaveDriver  --  CiA 402 EtherCAT slave driver for robot arm drives
// ---------------------------------------------------------------------------
class RobotSlaveDriver {
public:
    RobotSlaveDriver(const std::string& name,
                     uint16_t alias, uint16_t position,
                     uint32_t vendor_id, uint32_t product_code);
    ~RobotSlaveDriver() = default;

    // ----- Configuration (called before master activation) -----
    bool configure(ec_master_t* master, ec_domain_t* domain);
    void register_pdo_entries(ec_domain_t* domain);

    // ----- Cyclic update (called from RT thread) -----
    void read_inputs(uint8_t* domain_data);
    void write_outputs(uint8_t* domain_data);

    // ----- CiA 402 state machine -----
    bool   enable_operation();
    bool   disable_operation();
    bool   quick_stop();
    bool   fault_reset();
    DriveState current_drive_state() const;

    // ----- Mode selection -----
    void set_operation_mode(OperationMode mode);
    OperationMode current_operation_mode() const;

    // ----- Motion commands (set before write_outputs) -----
    void set_target_position(int32_t counts);
    void set_target_velocity(int32_t counts_per_sec);
    void set_target_torque(int16_t permille);   // 0.1% rated torque

    // ----- Feedback -----
    int32_t actual_position()  const { return actual_position_.load(); }
    int32_t actual_velocity()  const { return actual_velocity_.load(); }
    int16_t actual_torque()    const { return actual_torque_.load(); }
    uint16_t statusword()      const { return statusword_.load(); }
    uint32_t digital_inputs()  const { return digital_inputs_.load(); }

    // ----- Limits -----
    void set_position_limits(int32_t min_counts, int32_t max_counts);
    void set_velocity_limit(uint32_t max_counts_per_sec);
    void set_torque_limit(uint16_t max_permille);

    // ----- Homing -----
    bool start_homing(int8_t method = 35);
    bool is_homing_complete() const;

    const std::string& name() const { return name_; }

private:
    DriveState decode_statusword(uint16_t sw) const;
    uint16_t   build_controlword(DriveState current, DriveState target) const;
    void       enforce_limits();

    std::string name_;
    uint16_t alias_;
    uint16_t position_;
    uint32_t vendor_id_;
    uint32_t product_code_;

    ec_slave_config_t* sc_ = nullptr;
    RobotPdoOffsets    off_{};
    bool               activated_ = false;

    // Commanded values (written by application, read by RT loop)
    std::atomic<uint16_t>      controlword_cmd_{0};
    std::atomic<int32_t>       target_position_{0};
    std::atomic<int32_t>       target_velocity_{0};
    std::atomic<int16_t>       target_torque_{0};
    std::atomic<OperationMode> requested_mode_{OperationMode::CSP};

    // Actual values (written by RT loop, read by application)
    std::atomic<uint16_t> statusword_{0};
    std::atomic<int32_t>  actual_position_{0};
    std::atomic<int32_t>  actual_velocity_{0};
    std::atomic<int16_t>  actual_torque_{0};
    std::atomic<uint32_t> digital_inputs_{0};

    // Limits
    int32_t  pos_min_   = INT32_MIN;
    int32_t  pos_max_   = INT32_MAX;
    uint32_t vel_max_   = UINT32_MAX;
    uint16_t torque_max_= 1000;   // 100.0%

    // CiA 402 state machine
    std::atomic<DriveState> drive_state_{DriveState::UNKNOWN};
    DriveState              target_state_{DriveState::SWITCH_ON_DISABLED};
    bool                    enable_requested_ = false;
    bool                    homing_active_    = false;
    std::atomic<bool>       fault_reset_pending_{false};
};
