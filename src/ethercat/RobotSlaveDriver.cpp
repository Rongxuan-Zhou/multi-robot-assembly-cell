#include "RobotSlaveDriver.h"

#include <algorithm>
#include <cstdio>
#include <cstring>

// ---------------------------------------------------------------------------
// CiA 402 controlword bit definitions
// ---------------------------------------------------------------------------
static constexpr uint16_t CW_SWITCH_ON        = 1 << 0;
static constexpr uint16_t CW_ENABLE_VOLTAGE   = 1 << 1;
static constexpr uint16_t CW_QUICK_STOP       = 1 << 2;
static constexpr uint16_t CW_ENABLE_OPERATION = 1 << 3;
static constexpr uint16_t CW_FAULT_RESET      = 1 << 7;
static constexpr uint16_t CW_HALT             = 1 << 8;
// Homing-specific bits
static constexpr uint16_t CW_HOMING_START     = 1 << 4;

// ---------------------------------------------------------------------------
// CiA 402 statusword bit masks
// ---------------------------------------------------------------------------
static constexpr uint16_t SW_READY_TO_SWITCH_ON = 1 << 0;
static constexpr uint16_t SW_SWITCHED_ON        = 1 << 1;
static constexpr uint16_t SW_OP_ENABLED         = 1 << 2;
static constexpr uint16_t SW_FAULT              = 1 << 3;
static constexpr uint16_t SW_VOLTAGE_ENABLED    = 1 << 4;
static constexpr uint16_t SW_QUICK_STOP         = 1 << 5;
static constexpr uint16_t SW_SWITCH_ON_DISABLED = 1 << 6;
static constexpr uint16_t SW_HOMING_ATTAINED    = 1 << 12;
static constexpr uint16_t SW_TARGET_REACHED     = 1 << 10;

// Statusword state mask (bits 0-3, 5, 6)
static constexpr uint16_t SW_STATE_MASK = 0x006F;

// ---------------------------------------------------------------------------
// PDO entry registration table  (index, subindex, bit_length)
// ---------------------------------------------------------------------------
static const ec_pdo_entry_reg_t* make_pdo_regs(
    uint16_t alias, uint16_t pos,
    uint32_t vid, uint32_t pid,
    RobotPdoOffsets* off)
{
    // We build these on the stack; IgH copies them during registration.
    static ec_pdo_entry_reg_t regs[13];  // 12 entries + sentinel
    size_t i = 0;

    regs[i++] = {alias, pos, vid, pid, 0x6040, 0, &off->controlword};
    regs[i++] = {alias, pos, vid, pid, 0x607A, 0, &off->target_position};
    regs[i++] = {alias, pos, vid, pid, 0x60FF, 0, &off->target_velocity};
    regs[i++] = {alias, pos, vid, pid, 0x6071, 0, &off->target_torque};
    regs[i++] = {alias, pos, vid, pid, 0x6060, 0, &off->mode_of_operation};
    regs[i++] = {alias, pos, vid, pid, 0x60FE, 1, &off->digital_outputs};
    regs[i++] = {alias, pos, vid, pid, 0x6041, 0, &off->statusword};
    regs[i++] = {alias, pos, vid, pid, 0x6064, 0, &off->actual_position};
    regs[i++] = {alias, pos, vid, pid, 0x606C, 0, &off->actual_velocity};
    regs[i++] = {alias, pos, vid, pid, 0x6077, 0, &off->actual_torque};
    regs[i++] = {alias, pos, vid, pid, 0x6061, 0, &off->mode_of_operation_display};
    regs[i++] = {alias, pos, vid, pid, 0x60FD, 0, &off->digital_inputs};
    regs[i]   = {};   // sentinel

    return regs;
}

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------
RobotSlaveDriver::RobotSlaveDriver(const std::string& name,
                                   uint16_t alias, uint16_t position,
                                   uint32_t vendor_id, uint32_t product_code)
    : name_(name)
    , alias_(alias)
    , position_(position)
    , vendor_id_(vendor_id)
    , product_code_(product_code)
{
}

// ---------------------------------------------------------------------------
// configure  --  MUST be called before master activation.
//                PDO entries are registered per-slave here, and the IgH
//                master finalises all domain mappings during activate().
//                Calling configure() after activation is a fatal error.
// ---------------------------------------------------------------------------
bool RobotSlaveDriver::configure(ec_master_t* master, ec_domain_t* domain)
{
    if (activated_) {
        fprintf(stderr, "[%s] ERROR: Cannot configure after master activation\n",
                name_.c_str());
        return false;
    }

    sc_ = ecrt_master_slave_config(master, alias_, position_,
                                   vendor_id_, product_code_);
    if (!sc_) {
        fprintf(stderr, "[%s] Slave config failed\n", name_.c_str());
        return false;
    }

    // PDO mapping (CiA 402 standard mapping)
    ec_pdo_entry_info_t rx_entries[] = {
        {0x6040, 0x00, 16},   // controlword
        {0x607A, 0x00, 32},   // target position
        {0x60FF, 0x00, 32},   // target velocity
        {0x6071, 0x00, 16},   // target torque
        {0x6060, 0x00,  8},   // mode of operation
        {0x60FE, 0x01, 32},   // digital outputs
    };
    ec_pdo_entry_info_t tx_entries[] = {
        {0x6041, 0x00, 16},   // statusword
        {0x6064, 0x00, 32},   // actual position
        {0x606C, 0x00, 32},   // actual velocity
        {0x6077, 0x00, 16},   // actual torque
        {0x6061, 0x00,  8},   // mode of operation display
        {0x60FD, 0x00, 32},   // digital inputs
    };

    ec_pdo_info_t rx_pdos[] = {{0x1600, 6, rx_entries}};
    ec_pdo_info_t tx_pdos[] = {{0x1A00, 6, tx_entries}};

    ec_sync_info_t sync_info[] = {
        {0, EC_DIR_OUTPUT, 0, nullptr,  EC_WD_DISABLE},
        {1, EC_DIR_INPUT,  0, nullptr,  EC_WD_DISABLE},
        {2, EC_DIR_OUTPUT, 1, rx_pdos,  EC_WD_ENABLE},
        {3, EC_DIR_INPUT,  1, tx_pdos,  EC_WD_DISABLE},
        {0xFF}
    };

    if (ecrt_slave_config_pdos(sc_, EC_END, sync_info)) {
        fprintf(stderr, "[%s] PDO configuration failed\n", name_.c_str());
        return false;
    }

    // Register PDO entries in the domain
    const ec_pdo_entry_reg_t* regs = make_pdo_regs(
        alias_, position_, vendor_id_, product_code_, &off_);

    if (ecrt_domain_reg_pdo_entry_list(domain, regs)) {
        fprintf(stderr, "[%s] PDO entry registration failed\n", name_.c_str());
        return false;
    }

    printf("[%s] Configured (alias=%u, pos=%u)\n",
           name_.c_str(), alias_, position_);
    return true;
}

// ---------------------------------------------------------------------------
// read_inputs  --  copy TxPDO data to atomic feedback variables
// ---------------------------------------------------------------------------
void RobotSlaveDriver::read_inputs(uint8_t* domain_data)
{
    uint16_t sw = EC_READ_U16(domain_data + off_.statusword);
    statusword_.store(sw, std::memory_order_relaxed);
    drive_state_.store(decode_statusword(sw), std::memory_order_relaxed);

    // CiA 402 fault reset requires a rising edge on bit 7.
    // After the previous cycle wrote CW_FAULT_RESET, clear it here
    // so the next write_outputs() produces the falling edge.
    if (fault_reset_pending_.load(std::memory_order_relaxed)) {
        uint16_t cw = controlword_cmd_.load(std::memory_order_relaxed);
        if (cw & CW_FAULT_RESET) {
            controlword_cmd_.store(0, std::memory_order_relaxed);
            fault_reset_pending_.store(false, std::memory_order_relaxed);
        }
    }

    actual_position_.store(
        EC_READ_S32(domain_data + off_.actual_position),
        std::memory_order_relaxed);
    actual_velocity_.store(
        EC_READ_S32(domain_data + off_.actual_velocity),
        std::memory_order_relaxed);
    actual_torque_.store(
        EC_READ_S16(domain_data + off_.actual_torque),
        std::memory_order_relaxed);
    digital_inputs_.store(
        EC_READ_U32(domain_data + off_.digital_inputs),
        std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// write_outputs  --  write RxPDO data from commanded values
// ---------------------------------------------------------------------------
void RobotSlaveDriver::write_outputs(uint8_t* domain_data)
{
    enforce_limits();

    EC_WRITE_U16(domain_data + off_.controlword,
                 controlword_cmd_.load(std::memory_order_relaxed));
    EC_WRITE_S32(domain_data + off_.target_position,
                 target_position_.load(std::memory_order_relaxed));
    EC_WRITE_S32(domain_data + off_.target_velocity,
                 target_velocity_.load(std::memory_order_relaxed));
    EC_WRITE_S16(domain_data + off_.target_torque,
                 target_torque_.load(std::memory_order_relaxed));
    EC_WRITE_S8(domain_data + off_.mode_of_operation,
                static_cast<int8_t>(
                    requested_mode_.load(std::memory_order_relaxed)));
}

// ---------------------------------------------------------------------------
// CiA 402 state machine
// ---------------------------------------------------------------------------
DriveState RobotSlaveDriver::decode_statusword(uint16_t sw) const
{
    uint16_t masked = sw & SW_STATE_MASK;

    // Order matters: check more specific masks first
    if ((masked & 0x004F) == 0x0040) return DriveState::SWITCH_ON_DISABLED;
    if ((masked & 0x006F) == 0x0021) return DriveState::READY_TO_SWITCH_ON;
    if ((masked & 0x006F) == 0x0023) return DriveState::SWITCHED_ON;
    if ((masked & 0x006F) == 0x0027) return DriveState::OPERATION_ENABLED;
    if ((masked & 0x006F) == 0x0007) return DriveState::QUICK_STOP_ACTIVE;
    if ((masked & 0x004F) == 0x000F) return DriveState::FAULT_REACTION_ACTIVE;
    if ((masked & 0x004F) == 0x0008) return DriveState::FAULT;
    if ((masked & 0x004F) == 0x0000) return DriveState::NOT_READY_TO_SWITCH_ON;

    return DriveState::UNKNOWN;
}

bool RobotSlaveDriver::enable_operation()
{
    DriveState ds = drive_state_.load(std::memory_order_relaxed);

    switch (ds) {
        case DriveState::SWITCH_ON_DISABLED:
            // -> READY_TO_SWITCH_ON: set shutdown command
            controlword_cmd_.store(CW_ENABLE_VOLTAGE | CW_QUICK_STOP,
                                   std::memory_order_relaxed);
            break;

        case DriveState::READY_TO_SWITCH_ON:
            // -> SWITCHED_ON: set switch on command
            controlword_cmd_.store(CW_SWITCH_ON | CW_ENABLE_VOLTAGE | CW_QUICK_STOP,
                                   std::memory_order_relaxed);
            break;

        case DriveState::SWITCHED_ON:
            // -> OPERATION_ENABLED: set enable operation command
            controlword_cmd_.store(CW_SWITCH_ON | CW_ENABLE_VOLTAGE |
                                   CW_QUICK_STOP | CW_ENABLE_OPERATION,
                                   std::memory_order_relaxed);
            break;

        case DriveState::OPERATION_ENABLED:
            return true;  // already there

        case DriveState::FAULT:
            // Must fault_reset first
            return false;

        case DriveState::QUICK_STOP_ACTIVE:
            // -> SWITCH_ON_DISABLED (via QS completion), then re-enable
            controlword_cmd_.store(0, std::memory_order_relaxed);
            break;

        default:
            break;
    }

    enable_requested_ = true;
    return false;
}

bool RobotSlaveDriver::disable_operation()
{
    controlword_cmd_.store(CW_ENABLE_VOLTAGE | CW_QUICK_STOP,
                           std::memory_order_relaxed);
    enable_requested_ = false;
    target_state_ = DriveState::SWITCHED_ON;
    return true;
}

bool RobotSlaveDriver::quick_stop()
{
    // Remove Quick Stop bit -> drive executes quick stop ramp
    uint16_t cw = controlword_cmd_.load(std::memory_order_relaxed);
    cw &= ~CW_QUICK_STOP;
    controlword_cmd_.store(cw, std::memory_order_relaxed);
    enable_requested_ = false;
    return true;
}

bool RobotSlaveDriver::fault_reset()
{
    DriveState ds = drive_state_.load(std::memory_order_relaxed);
    if (ds != DriveState::FAULT) return false;

    // Rising edge on fault reset bit: set the bit now; the next
    // read_inputs() cycle will clear it to complete the rising edge.
    controlword_cmd_.store(CW_FAULT_RESET, std::memory_order_relaxed);
    fault_reset_pending_.store(true, std::memory_order_relaxed);
    return true;
}

DriveState RobotSlaveDriver::current_drive_state() const
{
    return drive_state_.load(std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// Mode selection
// ---------------------------------------------------------------------------
void RobotSlaveDriver::set_operation_mode(OperationMode mode)
{
    requested_mode_.store(mode, std::memory_order_relaxed);
}

OperationMode RobotSlaveDriver::current_operation_mode() const
{
    return requested_mode_.load(std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// Motion commands
// ---------------------------------------------------------------------------
void RobotSlaveDriver::set_target_position(int32_t counts)
{
    target_position_.store(counts, std::memory_order_relaxed);
}

void RobotSlaveDriver::set_target_velocity(int32_t counts_per_sec)
{
    target_velocity_.store(counts_per_sec, std::memory_order_relaxed);
}

void RobotSlaveDriver::set_target_torque(int16_t permille)
{
    target_torque_.store(permille, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// Limits
// ---------------------------------------------------------------------------
void RobotSlaveDriver::set_position_limits(int32_t min_counts, int32_t max_counts)
{
    pos_min_ = min_counts;
    pos_max_ = max_counts;
}

void RobotSlaveDriver::set_velocity_limit(uint32_t max_counts_per_sec)
{
    vel_max_ = max_counts_per_sec;
}

void RobotSlaveDriver::set_torque_limit(uint16_t max_permille)
{
    torque_max_ = max_permille;
}

void RobotSlaveDriver::enforce_limits()
{
    // Position clamping
    int32_t pos = target_position_.load(std::memory_order_relaxed);
    pos = std::clamp(pos, pos_min_, pos_max_);
    target_position_.store(pos, std::memory_order_relaxed);

    // Velocity clamping
    int32_t vel = target_velocity_.load(std::memory_order_relaxed);
    int32_t vel_lim = static_cast<int32_t>(vel_max_);
    vel = std::clamp(vel, -vel_lim, vel_lim);
    target_velocity_.store(vel, std::memory_order_relaxed);

    // Torque clamping
    int16_t trq = target_torque_.load(std::memory_order_relaxed);
    int16_t trq_lim = static_cast<int16_t>(torque_max_);
    trq = std::clamp(trq, static_cast<int16_t>(-trq_lim), trq_lim);
    target_torque_.store(trq, std::memory_order_relaxed);
}

// ---------------------------------------------------------------------------
// Homing
// ---------------------------------------------------------------------------
bool RobotSlaveDriver::start_homing(int8_t method)
{
    if (drive_state_.load() != DriveState::OPERATION_ENABLED) {
        return false;
    }

    // Set homing mode
    requested_mode_.store(OperationMode::HOMING, std::memory_order_relaxed);
    homing_active_ = true;

    // Write homing method via SDO would be done externally:
    //   SDO 0x6098:00 = method (e.g. 35 = current threshold)
    // Start homing by setting bit 4
    uint16_t cw = controlword_cmd_.load(std::memory_order_relaxed);
    cw |= CW_HOMING_START;
    controlword_cmd_.store(cw, std::memory_order_relaxed);

    printf("[%s] Homing started (method %d)\n", name_.c_str(), method);
    return true;
}

bool RobotSlaveDriver::is_homing_complete() const
{
    uint16_t sw = statusword_.load(std::memory_order_relaxed);
    // Homing attained (bit 12) and target reached (bit 10)
    return (sw & SW_HOMING_ATTAINED) && (sw & SW_TARGET_REACHED);
}
