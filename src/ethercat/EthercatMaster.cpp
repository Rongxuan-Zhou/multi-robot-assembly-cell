#include "EthercatMaster.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <ctime>
#include <sched.h>
#include <sys/mman.h>
#include <unistd.h>

// ---------------------------------------------------------------------------
// Slave catalog for this assembly cell
// ---------------------------------------------------------------------------
static const SlaveDescriptor kSlaveTable[] = {
    {0, 0, 0x000001DD, 0x10305070, "R1_FANUC_Drive"   },  // FANUC servo
    {0, 1, 0x00000539, 0x02010100, "R2_ROKAE_Drive"   },  // ROKAE servo
    {0, 2, 0x00000539, 0x02010100, "R3_ROKAE_Drive"   },  // ROKAE servo
    {0, 3, 0x00000002, 0x044C2C52, "Turntable_Servo"  },  // Beckhoff AX5206
    {0, 4, 0x00000002, 0x044C2C52, "ScrewDriver_1"    },  // spindle drive
    {0, 5, 0x00000002, 0x044C2C52, "ScrewDriver_2"    },  // spindle drive
    {0, 6, 0x00000002, 0x04622C22, "RemoteIO_EL1809"  },  // 16-ch DI
    {0, 7, 0x00000002, 0x07D22C22, "RemoteIO_EL2809"  },  // 16-ch DO
    {0, 8, 0x00000002, 0x04562C22, "SafetyIO_EL6910"  },  // TwinSAFE
};
static constexpr size_t kSlaveCount = sizeof(kSlaveTable) / sizeof(kSlaveTable[0]);

// ---------------------------------------------------------------------------
// Helper: timespec arithmetic
// ---------------------------------------------------------------------------
static inline void timespec_add_ns(struct timespec* ts, int64_t ns)
{
    ts->tv_nsec += ns;
    while (ts->tv_nsec >= 1'000'000'000L) {
        ts->tv_nsec -= 1'000'000'000L;
        ts->tv_sec++;
    }
    while (ts->tv_nsec < 0) {
        ts->tv_nsec += 1'000'000'000L;
        ts->tv_sec--;
    }
}

static inline int64_t timespec_diff_ns(const struct timespec* a,
                                        const struct timespec* b)
{
    return static_cast<int64_t>(a->tv_sec  - b->tv_sec) * 1'000'000'000L
         + static_cast<int64_t>(a->tv_nsec - b->tv_nsec);
}

// ---------------------------------------------------------------------------
// Construction / Destruction
// ---------------------------------------------------------------------------
// 1ms cycle time (1000us), suitable for multi-robot assembly cell
EthercatMaster::EthercatMaster(unsigned master_index, uint32_t cycle_time_us)
    : cycle_time_us_(cycle_time_us)
    , master_index_(master_index)
{
}

EthercatMaster::~EthercatMaster()
{
    request_stop();
    wait_for_stop();

    if (master_) {
        ecrt_release_master(master_);
        master_ = nullptr;
    }
}

// ---------------------------------------------------------------------------
// initialize -- request master from kernel module
// ---------------------------------------------------------------------------
bool EthercatMaster::initialize()
{
    master_ = ecrt_request_master(master_index_);
    if (!master_) {
        fprintf(stderr, "[EthercatMaster] Failed to request master %u\n",
                master_index_);
        return false;
    }

    domain_ = ecrt_master_create_domain(master_);
    if (!domain_) {
        fprintf(stderr, "[EthercatMaster] Failed to create process domain\n");
        return false;
    }

    printf("[EthercatMaster] Master %u initialized, domain created\n",
           master_index_);
    return true;
}

// ---------------------------------------------------------------------------
// configure_slaves -- set up PDO mappings and DC for all slaves
// ---------------------------------------------------------------------------
bool EthercatMaster::configure_slaves()
{
    slave_cfgs_.reserve(kSlaveCount);
    slave_descs_.reserve(kSlaveCount);

    for (size_t i = 0; i < kSlaveCount; ++i) {
        const auto& desc = kSlaveTable[i];
        ec_slave_config_t* sc = ecrt_master_slave_config(
            master_, desc.alias, desc.position,
            desc.vendor_id, desc.product_code);

        if (!sc) {
            fprintf(stderr, "[EthercatMaster] Failed to configure slave %s "
                    "(pos %u)\n", desc.name.c_str(), desc.position);
            return false;
        }

        // --- CiA 402 drives: configure PDO mapping (slots 0-5) ---
        if (i <= 5) {
            // RxPDO assignment (0x1600)
            ec_pdo_entry_info_t rx_entries[] = {
                {0x6040, 0x00, 16},  // controlword
                {0x607A, 0x00, 32},  // target position
                {0x60FF, 0x00, 32},  // target velocity
                {0x6071, 0x00, 16},  // target torque
                {0x6060, 0x00,  8},  // mode of operation
                {0x60FE, 0x01, 32},  // digital outputs
            };
            ec_pdo_info_t rx_pdos[] = {
                {0x1600, 6, rx_entries},
            };

            // TxPDO assignment (0x1A00)
            ec_pdo_entry_info_t tx_entries[] = {
                {0x6041, 0x00, 16},  // statusword
                {0x6064, 0x00, 32},  // actual position
                {0x606C, 0x00, 32},  // actual velocity
                {0x6077, 0x00, 16},  // actual torque
                {0x6061, 0x00,  8},  // mode of operation display
                {0x60FD, 0x00, 32},  // digital inputs
            };
            ec_pdo_info_t tx_pdos[] = {
                {0x1A00, 6, tx_entries},
            };

            ec_sync_info_t sync_info[] = {
                {0, EC_DIR_OUTPUT, 0, nullptr, EC_WD_DISABLE},
                {1, EC_DIR_INPUT,  0, nullptr, EC_WD_DISABLE},
                {2, EC_DIR_OUTPUT, 1, rx_pdos, EC_WD_ENABLE},
                {3, EC_DIR_INPUT,  1, tx_pdos, EC_WD_DISABLE},
                {0xFF}
            };

            if (ecrt_slave_config_pdos(sc, EC_END, sync_info)) {
                fprintf(stderr, "[EthercatMaster] PDO config failed for %s\n",
                        desc.name.c_str());
                return false;
            }

            // Distributed Clocks: SYNC0, cycle_time_us_ in nanoseconds
            ecrt_slave_config_dc(sc, 0x0300,
                                 cycle_time_us_ * 1000,  // SYNC0 cycle (ns)
                                 0,                       // SYNC0 shift
                                 0, 0);                   // SYNC1 off
        }

        slave_cfgs_.push_back(sc);
        slave_descs_.push_back(desc);

        printf("[EthercatMaster] Configured slave %zu: %s\n",
               i, desc.name.c_str());
    }

    return true;
}

// ---------------------------------------------------------------------------
// activate -- transition master to active state, lock pages
// ---------------------------------------------------------------------------
bool EthercatMaster::activate()
{
    if (ecrt_master_activate(master_)) {
        fprintf(stderr, "[EthercatMaster] Master activation failed\n");
        return false;
    }

    domain_data_ = ecrt_domain_data(domain_);
    if (!domain_data_) {
        fprintf(stderr, "[EthercatMaster] Failed to get domain data ptr\n");
        return false;
    }

    // Lock all current and future pages to prevent page faults in RT
    if (mlockall(MCL_CURRENT | MCL_FUTURE) == -1) {
        fprintf(stderr, "[EthercatMaster] mlockall failed: %s\n",
                strerror(errno));
        // Non-fatal but bad for determinism
    }

    printf("[EthercatMaster] Activated. Domain data at %p\n",
           static_cast<void*>(domain_data_));
    return true;
}

// ---------------------------------------------------------------------------
// RT thread entry point (static trampoline)
// ---------------------------------------------------------------------------
static void* rt_thread_entry(void* arg)
{
    auto* self = static_cast<EthercatMaster*>(arg);
    self->cyclic_task();          // never returns until running_ == false
    return nullptr;
}

// ---------------------------------------------------------------------------
// start_cyclic_task -- spawn SCHED_FIFO thread on isolated core
// ---------------------------------------------------------------------------
void EthercatMaster::start_cyclic_task(int cpu_core)
{
    running_.store(true);

    pthread_attr_t attr;
    pthread_attr_init(&attr);

    // Stack: 512 KB pre-allocated (avoid page faults)
    pthread_attr_setstacksize(&attr, 512 * 1024);

    // SCHED_FIFO priority 98 (just below watchdog)
    struct sched_param param{};
    param.sched_priority = 98;
    pthread_attr_setschedpolicy(&attr, SCHED_FIFO);
    pthread_attr_setschedparam(&attr, &param);
    pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED);

    int ret = pthread_create(&rt_thread_, &attr, rt_thread_entry, this);
    if (ret != 0) {
        fprintf(stderr, "[EthercatMaster] pthread_create failed: %s\n",
                strerror(ret));
        running_.store(false);
        pthread_attr_destroy(&attr);
        return;
    }

    // CPU affinity: pin to isolated core
    if (cpu_core >= 0) {
        cpu_set_t cpuset;
        CPU_ZERO(&cpuset);
        CPU_SET(cpu_core, &cpuset);
        ret = pthread_setaffinity_np(rt_thread_, sizeof(cpuset), &cpuset);
        if (ret != 0) {
            fprintf(stderr, "[EthercatMaster] CPU affinity failed: %s\n",
                    strerror(ret));
        }
    }

    pthread_attr_destroy(&attr);
    printf("[EthercatMaster] Cyclic task started (core=%d, period=%u us)\n",
           cpu_core, cycle_time_us_);
}

// ---------------------------------------------------------------------------
// request_stop / wait_for_stop
// ---------------------------------------------------------------------------
void EthercatMaster::request_stop()
{
    running_.store(false);
}

void EthercatMaster::wait_for_stop()
{
    if (rt_thread_) {
        pthread_join(rt_thread_, nullptr);
        rt_thread_ = 0;
    }
}

// ---------------------------------------------------------------------------
// cyclic_task  --  deterministic real-time loop
// ---------------------------------------------------------------------------
void EthercatMaster::cyclic_task()
{
    struct timespec wakeup_time;
    clock_gettime(CLOCK_MONOTONIC, &wakeup_time);

    const int64_t period_ns = static_cast<int64_t>(cycle_time_us_) * 1000;
    int64_t jitter_sum   = 0;
    int64_t jitter_min   = INT64_MAX;
    int64_t jitter_max   = INT64_MIN;
    uint64_t cycles      = 0;
    uint64_t overruns    = 0;

    while (running_.load(std::memory_order_relaxed)) {
        // Sleep until next period
        timespec_add_ns(&wakeup_time, period_ns);
        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &wakeup_time, nullptr);

        // Measure jitter
        struct timespec now;
        clock_gettime(CLOCK_MONOTONIC, &now);
        int64_t jitter = timespec_diff_ns(&now, &wakeup_time);

        if (jitter < jitter_min) jitter_min = jitter;
        if (jitter > jitter_max) jitter_max = jitter;
        jitter_sum += jitter;
        ++cycles;

        if (jitter > period_ns / 2) {
            ++overruns;
        }

        // Update statistics every 1000 cycles (lock-free)
        if ((cycles & 0x3FF) == 0) {
            stats_.min_jitter_ns.store(jitter_min, std::memory_order_relaxed);
            stats_.max_jitter_ns.store(jitter_max, std::memory_order_relaxed);
            stats_.avg_jitter_ns.store(
                jitter_sum / static_cast<int64_t>(cycles),
                std::memory_order_relaxed);
            stats_.cycle_count.store(cycles, std::memory_order_relaxed);
            stats_.overrun_count.store(overruns, std::memory_order_relaxed);
        }

        // ----- EtherCAT cycle -----
        // 1. Receive process data
        ecrt_master_receive(master_);
        ecrt_domain_process(domain_);

        // 2. Check domain / master / slave states periodically
        if ((cycles % 500) == 0) {
            check_master_state();
            check_slave_states();
        }

        // 3. Handle emergency messages
        handle_emergency_messages();

        // 4. Application-level processing happens here via registered
        //    slave drivers' read_inputs / write_outputs, called by the
        //    application layer between receive and send.
        //    (The master exposes domain_data_ for slave drivers to use.)

        // 5. Sync reference clock (DC)
        ecrt_master_sync_reference_clock(master_);
        ecrt_master_sync_slave_clocks(master_);

        // 6. Queue and send process data
        ecrt_domain_queue(domain_);
        ecrt_master_send(master_);
    }

    printf("[EthercatMaster] Cyclic task exiting after %lu cycles "
           "(overruns: %lu)\n", cycles, overruns);
}

// ---------------------------------------------------------------------------
// check_master_state
// ---------------------------------------------------------------------------
void EthercatMaster::check_master_state()
{
    ec_master_state_t ms;
    ecrt_master_state(master_, &ms);

    if (ms.slaves_responding != master_st_.slaves_responding) {
        printf("[EthercatMaster] Slaves responding: %u -> %u\n",
               master_st_.slaves_responding, ms.slaves_responding);
    }
    if (ms.al_states != master_st_.al_states) {
        printf("[EthercatMaster] AL states: 0x%02X -> 0x%02X\n",
               master_st_.al_states, ms.al_states);
    }
    if (ms.link_up != master_st_.link_up) {
        printf("[EthercatMaster] Link: %s\n",
               ms.link_up ? "UP" : "DOWN");
    }
    master_st_ = ms;
}

// ---------------------------------------------------------------------------
// check_slave_states
// ---------------------------------------------------------------------------
void EthercatMaster::check_slave_states()
{
    for (size_t i = 0; i < slave_cfgs_.size(); ++i) {
        ec_slave_config_state_t s;
        ecrt_slave_config_state(slave_cfgs_[i], &s);

        if (s.al_state == 0x08) continue;  // OP is fine

        if (s.al_state == 0x01 || s.al_state == 0x02) {
            fprintf(stderr, "[EthercatMaster] Slave %zu (%s) in state 0x%02X\n",
                    i, slave_descs_[i].name.c_str(), s.al_state);
        }
    }
}

// ---------------------------------------------------------------------------
// handle_emergency_messages
// ---------------------------------------------------------------------------
void EthercatMaster::handle_emergency_messages()
{
    // IgH master exposes emergencies via the SDO/mailbox interface.
    // In a production system we would poll ecrt_master_sdo_upload for
    // CoE emergency objects.  Here we show the callback dispatch pattern.
    //
    // The actual emergency data would come from slave mailbox processing.
    // This is a placeholder showing the integration point.
}

// ---------------------------------------------------------------------------
// SDO read / write  (non-RT, blocking)
// ---------------------------------------------------------------------------
int EthercatMaster::sdo_read(uint16_t slave_pos, uint16_t index,
                              uint8_t subindex, uint8_t* data, size_t size)
{
    size_t result_size = size;
    uint32_t abort_code = 0;
    int ret = ecrt_master_sdo_upload(master_, slave_pos,
                                      index, subindex,
                                      data, size,
                                      &result_size, &abort_code);
    if (ret < 0) {
        fprintf(stderr, "[EthercatMaster] SDO read 0x%04X:%02X failed "
                "(abort 0x%08X)\n", index, subindex, abort_code);
    }
    return ret;
}

int EthercatMaster::sdo_write(uint16_t slave_pos, uint16_t index,
                               uint8_t subindex,
                               const uint8_t* data, size_t size)
{
    uint32_t abort_code = 0;
    int ret = ecrt_master_sdo_download(master_, slave_pos,
                                        index, subindex,
                                        data, size,
                                        &abort_code);
    if (ret < 0) {
        fprintf(stderr, "[EthercatMaster] SDO write 0x%04X:%02X failed "
                "(abort 0x%08X)\n", index, subindex, abort_code);
    }
    return ret;
}

// ---------------------------------------------------------------------------
// master_state
// ---------------------------------------------------------------------------
SlaveState EthercatMaster::master_state() const
{
    switch (master_st_.al_states) {
        case 0x01: return SlaveState::INIT;
        case 0x02: return SlaveState::PREOP;
        case 0x04: return SlaveState::SAFEOP;
        case 0x08: return SlaveState::OP;
        default:   return SlaveState::UNKNOWN;
    }
}

// ---------------------------------------------------------------------------
// set_emergency_callback
// ---------------------------------------------------------------------------
void EthercatMaster::set_emergency_callback(EmergencyCallback cb)
{
    std::lock_guard<std::mutex> lk(emg_mutex_);
    emg_cb_ = std::move(cb);
}
