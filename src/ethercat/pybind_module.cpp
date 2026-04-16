#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/functional.h>

#include "EthercatMaster.h"
#include "RobotSlaveDriver.h"
#include "TurntableController.h"
#include "ScrewDriverController.h"
#include "SafetyMonitor.h"

namespace py = pybind11;

PYBIND11_MODULE(ethercat_py, m) {
    m.doc() = "Multi-robot assembly cell EtherCAT control layer";

    // -----------------------------------------------------------------------
    // Enums
    // -----------------------------------------------------------------------
    py::enum_<SlaveState>(m, "SlaveState")
        .value("INIT",    SlaveState::INIT)
        .value("PREOP",   SlaveState::PREOP)
        .value("SAFEOP",  SlaveState::SAFEOP)
        .value("OP",      SlaveState::OP)
        .value("UNKNOWN", SlaveState::UNKNOWN);

    py::enum_<OperationMode>(m, "OperationMode")
        .value("NO_MODE", OperationMode::NO_MODE)
        .value("PP",      OperationMode::PP)
        .value("PV",      OperationMode::PV)
        .value("PT",      OperationMode::PT)
        .value("HOMING",  OperationMode::HOMING)
        .value("IP",      OperationMode::IP)
        .value("CSP",     OperationMode::CSP)
        .value("CSV",     OperationMode::CSV)
        .value("CST",     OperationMode::CST);

    py::enum_<DriveState>(m, "DriveState")
        .value("NOT_READY_TO_SWITCH_ON", DriveState::NOT_READY_TO_SWITCH_ON)
        .value("SWITCH_ON_DISABLED",     DriveState::SWITCH_ON_DISABLED)
        .value("READY_TO_SWITCH_ON",     DriveState::READY_TO_SWITCH_ON)
        .value("SWITCHED_ON",           DriveState::SWITCHED_ON)
        .value("OPERATION_ENABLED",      DriveState::OPERATION_ENABLED)
        .value("QUICK_STOP_ACTIVE",      DriveState::QUICK_STOP_ACTIVE)
        .value("FAULT_REACTION_ACTIVE",  DriveState::FAULT_REACTION_ACTIVE)
        .value("FAULT",                  DriveState::FAULT)
        .value("UNKNOWN",               DriveState::UNKNOWN);

    py::enum_<SafetyState>(m, "SafetyState")
        .value("RUN",             SafetyState::RUN)
        .value("SAFE_REDUCED",    SafetyState::SAFE_REDUCED)
        .value("SAFE_STOP_REQ",   SafetyState::SAFE_STOP_REQ)
        .value("SAFE_STOP_ACTIVE",SafetyState::SAFE_STOP_ACTIVE)
        .value("STO_ACTIVE",      SafetyState::STO_ACTIVE)
        .value("FAULT",           SafetyState::FAULT);

    py::enum_<Station>(m, "Station")
        .value("STATION_1", Station::STATION_1)
        .value("STATION_2", Station::STATION_2)
        .value("STATION_3", Station::STATION_3);

    py::enum_<TurntableState>(m, "TurntableState")
        .value("IDLE",          TurntableState::IDLE)
        .value("UNCLAMPING",    TurntableState::UNCLAMPING)
        .value("WAITING_CLEAR", TurntableState::WAITING_CLEAR)
        .value("ACCELERATING",  TurntableState::ACCELERATING)
        .value("CRUISE",        TurntableState::CRUISE)
        .value("DECELERATING",  TurntableState::DECELERATING)
        .value("SETTLING",      TurntableState::SETTLING)
        .value("CLAMPING",      TurntableState::CLAMPING)
        .value("IN_POSITION",   TurntableState::IN_POSITION)
        .value("FAULT",         TurntableState::FAULT);

    py::enum_<ScrewPhase>(m, "ScrewPhase")
        .value("IDLE",           ScrewPhase::IDLE)
        .value("FEEDER_ADVANCE", ScrewPhase::FEEDER_ADVANCE)
        .value("APPROACH",       ScrewPhase::APPROACH)
        .value("SNUG",           ScrewPhase::SNUG)
        .value("FINAL_TORQUE",   ScrewPhase::FINAL_TORQUE)
        .value("VERIFY",         ScrewPhase::VERIFY)
        .value("RETRACT",        ScrewPhase::RETRACT)
        .value("COMPLETE",       ScrewPhase::COMPLETE)
        .value("FAULT",          ScrewPhase::FAULT);

    // -----------------------------------------------------------------------
    // CycleStats (read-only snapshot)
    // -----------------------------------------------------------------------
    py::class_<CycleStats>(m, "CycleStats")
        .def_property_readonly("min_jitter_ns",
            [](const CycleStats& s) { return s.min_jitter_ns.load(); })
        .def_property_readonly("max_jitter_ns",
            [](const CycleStats& s) { return s.max_jitter_ns.load(); })
        .def_property_readonly("avg_jitter_ns",
            [](const CycleStats& s) { return s.avg_jitter_ns.load(); })
        .def_property_readonly("cycle_count",
            [](const CycleStats& s) { return s.cycle_count.load(); })
        .def_property_readonly("overrun_count",
            [](const CycleStats& s) { return s.overrun_count.load(); });

    // -----------------------------------------------------------------------
    // EthercatMaster
    // -----------------------------------------------------------------------
    py::class_<EthercatMaster>(m, "EthercatMaster")
        .def(py::init<unsigned, uint32_t>(),
             py::arg("master_index") = 0,
             py::arg("cycle_time_us") = 1000)
        .def("initialize",        &EthercatMaster::initialize)
        .def("configure_slaves",  &EthercatMaster::configure_slaves)
        .def("activate",          &EthercatMaster::activate)
        .def("start_cyclic_task", &EthercatMaster::start_cyclic_task,
             py::arg("cpu_core") = -1)
        .def("request_stop",      &EthercatMaster::request_stop)
        .def("wait_for_stop",     &EthercatMaster::wait_for_stop)
        .def("master_state",      &EthercatMaster::master_state)
        .def("cycle_stats",       &EthercatMaster::cycle_stats,
             py::return_value_policy::reference);

    // -----------------------------------------------------------------------
    // RobotSlaveDriver
    // -----------------------------------------------------------------------
    py::class_<RobotSlaveDriver>(m, "RobotSlaveDriver")
        .def(py::init<const std::string&, uint16_t, uint16_t,
                       uint32_t, uint32_t>(),
             py::arg("name"), py::arg("alias"), py::arg("position"),
             py::arg("vendor_id"), py::arg("product_code"))
        .def("enable_operation",      &RobotSlaveDriver::enable_operation)
        .def("disable_operation",     &RobotSlaveDriver::disable_operation)
        .def("quick_stop",            &RobotSlaveDriver::quick_stop)
        .def("fault_reset",           &RobotSlaveDriver::fault_reset)
        .def("current_drive_state",   &RobotSlaveDriver::current_drive_state)
        .def("set_operation_mode",    &RobotSlaveDriver::set_operation_mode)
        .def("current_operation_mode",&RobotSlaveDriver::current_operation_mode)
        .def("set_target_position",   &RobotSlaveDriver::set_target_position)
        .def("set_target_velocity",   &RobotSlaveDriver::set_target_velocity)
        .def("set_target_torque",     &RobotSlaveDriver::set_target_torque)
        .def("actual_position",       &RobotSlaveDriver::actual_position)
        .def("actual_velocity",       &RobotSlaveDriver::actual_velocity)
        .def("actual_torque",         &RobotSlaveDriver::actual_torque)
        .def("statusword",            &RobotSlaveDriver::statusword)
        .def("start_homing",          &RobotSlaveDriver::start_homing,
             py::arg("method") = 35)
        .def("is_homing_complete",    &RobotSlaveDriver::is_homing_complete)
        .def_property_readonly("name",&RobotSlaveDriver::name);

    // -----------------------------------------------------------------------
    // TurntableController
    // -----------------------------------------------------------------------
    py::class_<TurntableController>(m, "TurntableController")
        .def("move_to_station",    &TurntableController::move_to_station)
        .def("abort",              &TurntableController::abort)
        .def("state",              &TurntableController::state)
        .def("current_station",    &TurntableController::current_station)
        .def("in_position",        &TurntableController::in_position)
        .def("position_error_deg", &TurntableController::position_error_deg);

    // -----------------------------------------------------------------------
    // ScrewResult
    // -----------------------------------------------------------------------
    py::class_<ScrewResult>(m, "ScrewResult")
        .def_readonly("ok",           &ScrewResult::ok)
        .def_readonly("final_torque", &ScrewResult::final_torque)
        .def_readonly("final_angle",  &ScrewResult::final_angle)
        .def_readonly("duration_ms",  &ScrewResult::duration_ms)
        .def_readonly("screw_index",  &ScrewResult::screw_index);

    // -----------------------------------------------------------------------
    // SafetyMonitor
    // -----------------------------------------------------------------------
    py::class_<SafetyMonitor>(m, "SafetyMonitor")
        .def(py::init<>())
        .def("request_run",         &SafetyMonitor::request_run)
        .def("request_safe_stop",   &SafetyMonitor::request_safe_stop)
        .def("acknowledge_fault",   &SafetyMonitor::acknowledge_fault)
        .def("state",               &SafetyMonitor::state)
        .def("sto_active",          &SafetyMonitor::sto_active)
        .def("sls_active",          &SafetyMonitor::sls_active)
        .def("all_estops_ok",       &SafetyMonitor::all_estops_ok)
        .def("safety_door_closed",  &SafetyMonitor::safety_door_closed)
        .def("fault_code",          &SafetyMonitor::fault_code);
}
