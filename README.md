<div align="center">

# Multi-Robot Assembly Cell

### 3-Robot EtherCAT Assembly Cell for Harmonic Reducer Production

[![License: Apache 2.0](https://img.shields.io/badge/License-Apache%202.0-blue.svg)](LICENSE)
[![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-3776AB.svg?logo=python&logoColor=white)](https://www.python.org)
[![PLC: CODESYS](https://img.shields.io/badge/PLC-CODESYS%20Virtual%20Control%20SL-00599C.svg)](https://www.codesys.com)
[![Fieldbus: EtherCAT](https://img.shields.io/badge/Fieldbus-EtherCAT%201Gbps-E4002B.svg)](https://www.ethercat.org)
[![Robot: FANUC](https://img.shields.io/badge/Robot-FANUC-FFD700.svg)](https://www.fanuc.com)
[![Robot: ROKAE](https://img.shields.io/badge/Robot-ROKAE%20%E7%8F%9E%E7%9F%B3-00A651.svg)](https://www.rokae.com)
[![Safety: ISO 13849 PLd](https://img.shields.io/badge/Safety-ISO%2013849%20PLd-red.svg)](#technology-stack)

---

[Architecture](#system-architecture) · [Process Flow](#assembly-process) · [Documentation](#documentation) · [Source Code](#source-code) · [Tech Stack](#technology-stack)

</div>

<br/>

## Workstation Overview

<p align="center">
  <img src="docs/images/workstation_3d_layout.png" width="75%" alt="3D isometric view of assembly workstation"/>
</p>

<p align="center">
  <img src="docs/images/workstation_cad_topview.png" width="48%" alt="CAD top-down view showing R1/R2/R3 positions"/>
  <img src="docs/images/production_line_layout.png" width="48%" alt="Full production line with multiple workstations"/>
</p>

<p align="center">
  <em>Top left: 3D isometric view -- turntable, 3 robots (R1 FANUC, R2/R3 ROKAE), material racks, screw feeders.
  Top right: CAD top-down layout -- R1/R2/R3 positions, 120-degree turntable zones, conveyor interface.
  Bottom: Production-line floor plan with rows of standardized workstations (OP-8, OP-10 and OP-14 series).</em>
</p>

---

## Overview

This project reverse-engineers the complete automation system for an **OP08#-1 harmonic reducer actuator motor assembly workstation** originally deployed at CloudMinds Robotics. The workstation uses three coordinated robots around a 120-degree indexed turntable to assemble motor mounting bodies, amorphous motors, and brake magnet bases into harmonic reducer actuators (Actuator #8 and #11&#18) at a 94-second takt time.

The forward-looking architecture replaces the original Siemens SICAR stack with an open, Linux-based control platform built on **CODESYS Virtual Control SL** (containerized soft PLC) and **IgH EtherCAT Master** (kernel-space, PREEMPT_RT).

<table>
<tr><td>

**What it does**

- Assembles harmonic reducer actuator motors (Actuator #8 and #11&#18 variants)
- Coordinates 3 robots (1 FANUC + 2 ROKAE 6-axis) around a 120-degree indexed turntable
- Handles motor mounting body loading, motor placement, cable dressing, screw tightening, and brake magnet base assembly
- Supports product variant switching between Actuator #8 (4x M2.5x4 motor-mount screws) and #11&#18 (4x M2.5x6 motor-mount + 2x M2.5x6 brake magnet base screws)
- Integrates vision-guided picking, 3 screw feeders, and torque-controlled tightening with tightening vision

</td><td>

**How it works**

- R1 (FANUC) handles material: picks from stacker trays via vision, loads/unloads turntable station 1, assists tray changeover
- Turntable rotates 120 degrees to advance workpieces through 3 zones
- R2 (ROKAE) performs motor assembly on the shared workbench: motor placement, cable dressing, mounting body alignment, 4x screw tightening (M2.5x4 for #8, M2.5x6 for #11&#18)
- R3 (ROKAE) performs brake magnet base assembly (Actuator #11&#18 only): 2x M2.5x6 screws
- R2 returns the finished assembly from the workbench to turntable station 3; R1 unloads it at station 1 after the next index
- EtherCAT fieldbus synchronizes all axes at 1 ms cycle time

</td></tr>
</table>

### Key Specifications

| | |
|---|---|
| **Product** | Harmonic reducer actuator motor assembly -- Actuator #8 (motor mounting body L120 x W86.8 x H33.75 mm) and Actuator #11&#18 (L155 x W80 x H53.6 mm) |
| **Motors** | 40 mm amorphous motor (D47 x H17 mm) for #8, 60 mm amorphous axial motor (D68 x H20.8 mm) for #11&#18 |
| **Brake magnet base** | D20 x H24.55 mm, Actuator #11&#18 only |
| **Turntable** | 1-1.3m diameter flexible mother plate, 6 index plates x 3 sub-plates = 18 tooling positions, 120-degree indexing |
| **Takt Time** | 94 seconds (bottleneck: R2 assembly zone) -- Zone 1: 50s, Zone 2: 94s, Zone 3: 88s |
| **Robots** | 1x FANUC (material handling) + 2x ROKAE 6-axis (assembly process) |
| **Screws** | Motor mount: 4x M2.5x4 hex socket countersunk (#8, head dia 4.6 mm) or 4x M2.5x6 (#11&#18, head dia 5.6 mm); brake magnet base: 2x M2.5x6 (#11&#18 only) |
| **Screw feeding** | 3 screw feeders -- #1 (M2.5x4) and #2 (M2.5x6) at R2, #3 (M2.5x6) at R3; 2 torque-controlled drivers; 2 tightening cameras; 1 bit changer |
| **Control** | CODESYS Virtual Control SL on IPC (i7/Xeon, 32GB, PREEMPT_RT Linux) |
| **Fieldbus** | EtherCAT 1 Gbps, 1 ms cycle, IgH kernel-space master |
| **Safety** | FSoE (Fail Safe over EtherCAT) SIL3, ISO 13849-1 PLd, ISO 10218 robot safety zones |
| **Connectivity** | OPC UA for MES integration, MQTT for cloud telemetry |

### Station Equipment

Equipment list for station OP-8-1 as given in the source deck:

| Equipment | Qty | Equipment | Qty |
|:----------|:---:|:----------|:---:|
| Handling robot (R1, FANUC) | 1 | Process robots (6-axis, ROKAE: R2, R3) | 2 |
| Handling vision (R1 combo end-effector) | 1 | Tightening equipment | 2 |
| Flexible rotary table | 1 | Tightening vision | 2 |
| Workbench (shared by R2 / R3) | 1 | Screw feeders | 3 |
| Box stackers | 3 | Bit changer | 1 |
| Trays | 15 | Not used on OP-8-1: vibratory feeders, AUBO cobot, assembly vision, TM gear-meshing robot, press, laser welder | 0 |

---

## System Architecture

```mermaid
flowchart TB
    subgraph IPC["Industrial PC (i7/Xeon, 32GB, PREEMPT_RT Linux)"]
        CODESYS["<b>CODESYS Virtual Control SL</b><br/><i>Containerized IEC 61131-3 Soft PLC</i><br/><i>SoftMotion CNC/Robotics</i>"]
        IGH["<b>IgH EtherCAT Master</b><br/><i>Kernel module, DC sync</i><br/><i>1 ms cycle, &lt;1 us jitter</i>"]
        OPCUA["<b>OPC UA Server</b><br/><i>MES integration</i>"]
        MQTT["<b>MQTT Client</b><br/><i>Cloud telemetry</i>"]
    end

    subgraph ECAT["EtherCAT Network (1 Gbps)"]
        R1["<b>R1 -- FANUC</b><br/><i>Material handling</i><br/><i>Vision + gripper + suction</i>"]
        R2["<b>R2 -- ROKAE</b><br/><i>Motor assembly</i><br/><i>6-axis, screw tightening</i>"]
        R3["<b>R3 -- ROKAE</b><br/><i>Brake magnet assembly</i><br/><i>6-axis, screw tightening</i>"]
        TT["<b>Turntable Servo</b><br/><i>120-deg indexing</i>"]
        SF["<b>Screw Feeders (x3)</b><br/><i>#1 M2.5x4, #2 M2.5x6 (R2)</i><br/><i>#3 M2.5x6 (R3)</i>"]
        ST["<b>Screw Tightening (x2)</b><br/><i>Torque controlled</i>"]
        RIO["<b>Remote I/O</b><br/><i>Sensors, clamps, valves</i>"]
    end

    subgraph SAFETY["FSoE Safety Layer (SIL3)"]
        ESTOP["E-Stop"]
        LC["Light Curtains"]
        SD["Safety Doors"]
        SZ["Robot Safety Zones"]
    end

    subgraph VISION["Vision System"]
        V1["<b>R1 Vision</b><br/><i>Part localization</i><br/><i>Tray picking</i>"]
        V2["<b>R2 Tightening Vision</b><br/><i>Screw hole alignment</i>"]
        V3["<b>R3 Tightening Vision</b><br/><i>Screw hole alignment</i>"]
    end

    CODESYS --> IGH
    IGH --> R1
    IGH --> R2
    IGH --> R3
    IGH --> TT
    IGH --> SF
    IGH --> ST
    IGH --> RIO
    IGH -. "FSoE" .-> ESTOP
    IGH -. "FSoE" .-> LC
    IGH -. "FSoE" .-> SD
    IGH -. "FSoE" .-> SZ
    R1 --- V1
    R2 --- V2
    R3 --- V3
    CODESYS --> OPCUA
    CODESYS --> MQTT
```

---

## Assembly Process

The OP08#-1 workstation executes a 17-step assembly sequence across 3 turntable zones. Steps 12-16 are only executed for Actuator #11&#18 (with brake magnet base). Step 17 (R2 returns the finished assembly from the shared workbench to turntable station 3) is the last step of the cycle; see [Source Interpretation Notes](docs/process-flow.md#source-interpretation-notes) for how the source numbers it.

```mermaid
flowchart TD
    subgraph ZONE1["Zone 1 -- R1 Material Handling (50s)"]
        S1["1. R1 picks finished product<br/>from station 1 -> conveyor"]
        S2["2. R1 vision picks motor mount body<br/>from tray -> station 1"]
        S3["3. R1 vision picks motor<br/>from tray -> turntable transit"]
        S4["4. R1 vision picks brake magnet base<br/>(#11&#18 only) -> turntable transit"]
    end

    subgraph ROT1["Turntable Rotation 120 deg"]
        TR1["Turntable indexes 120 deg"]
    end

    subgraph ZONE2["Zone 2 -- R2 Motor Assembly (94s)"]
        S5["5. R2 picks motor<br/>from turntable -> workbench"]
        S6["6. Auxiliary cable-dressing fixture<br/>arranges motor cables"]
        S7["7. R2 picks motor mount body<br/>from turntable -> lowers onto motor<br/>(cable fixture retracts)"]
        S8["8. Clamp press motor / mount body"]
        S9["9. R2 picks screws from feeder 1 / 2<br/>M2.5x4 (#8) or M2.5x6 (#11&#18) x4"]
        S10["10. R2 auto-tighten screws"]
        S11["11. Release clamp"]
    end

    subgraph ROT2["Turntable Rotation 120 deg"]
        TR2["Turntable indexes 120 deg"]
    end

    subgraph ZONE3["Zone 3 -- R3 Brake Magnet Assembly (88s)"]
        S12["12. R3 picks brake magnet base<br/>(#11&#18 only) -> workbench"]
        S13["13. Clamp press"]
        S14["14. R3 picks screws from feeder 3<br/>M2.5x6 x2"]
        S15["15. R3 auto-tighten screws"]
        S16["16. Release clamp"]
        S17["17. R2 returns finished assembly<br/>workbench -> turntable station 3"]
    end

    S1 --> S2 --> S3 --> S4
    S4 --> TR1
    TR1 --> S5 --> S6 --> S7 --> S8 --> S9 --> S10 --> S11
    S11 --> TR2
    TR2 --> S12 --> S13 --> S14 --> S15 --> S16 --> S17
```

---

## Documentation

Detailed technical documentation covering the complete workstation design:

| Document | Topics | Key Content |
|:---------|:-------|:------------|
| **[System Architecture](docs/system-architecture.md)** | IPC, PLC, EtherCAT, vision, networking | CODESYS Virtual Control SL containerized runtime, IgH EtherCAT Master (PREEMPT_RT), DC synchronization, EtherCAT network topology, OPC UA / MQTT connectivity, vision system integration |
| **[Process Flow](docs/process-flow.md)** | 17-step assembly sequence, timing | Step-by-step OP08#-1 process, zone-by-zone cycle time analysis (50s / 94s / 88s), product variant switching, turntable indexing logic |
| **[Robot Coordination](docs/robot-coordination.md)** | Multi-robot control, interlocks, takt | 3-zone interlock logic, turntable rotation synchronization, collision avoidance, workspace overlap prevention, variant-based zone skipping |
| **[Safety Design](docs/safety-design.md)** | Functional safety, standards, zones | ISO 13849-1 PLd architecture, FSoE over EtherCAT (SIL3), E-Stop / light curtains / safety doors, ISO 10218 robot safety zones, collaborative workspace monitoring |

---

## Source Code

> **Python 3.10+ / C++ / IEC 61131-3 ST** -- Control software implementing multi-robot coordination, EtherCAT communication, vision-guided assembly, and PLC logic for the OP08#-1 workstation.

| Module | File | Description |
|:-------|:-----|:------------|
| **PLC** | [`main_sequence.st`](src/plc/main_sequence.st) | IEC 61131-3 Structured Text main cyclic program -- three-zone assembly state machine, variant branching, turntable handshake |
| **Orchestration** | [`CellOrchestrator.py`](tools/orchestration/CellOrchestrator.py) | Master controller -- cell state machine, zone-based parallel execution, turntable synchronization |
| | [`RobotInterface.py`](tools/orchestration/RobotInterface.py) | Unified abstraction for FANUC and ROKAE -- motion, tool control, vision-guided picking, force-controlled placement |
| | [`MaterialHandler.py`](tools/orchestration/MaterialHandler.py) | Material flow management -- tray stackers, conveyor belts, C-shape rack, part tracking |
| **Analysis** | [`ScrewTighteningManager.py`](tools/analysis/ScrewTighteningManager.py) | Multi-step screw driving -- M2.5x4 / M2.5x6 motor-mount screws (R2, per variant) and M2.5x6 brake magnet base screws (R3), torque/angle monitoring |
| **Vision** | [`VisionSystem.py`](tools/vision/VisionSystem.py) | 2D machine vision -- part localization, screw hole detection, tray-slot occupancy, GigE Vision / USB3 |
| **EtherCAT** | [`EthercatMaster.cpp`](src/ethercat/EthercatMaster.cpp) / [`.h`](src/ethercat/EthercatMaster.h) | IgH EtherCAT Master wrapper -- domain configuration, PDO exchange, distributed clock synchronization |
| | [`RobotSlaveDriver.cpp`](src/ethercat/RobotSlaveDriver.cpp) / [`.h`](src/ethercat/RobotSlaveDriver.h) | Robot EtherCAT slave driver -- CiA 402 profile, command/status PDO mapping for FANUC and ROKAE |
| | [`TurntableController.cpp`](src/ethercat/TurntableController.cpp) / [`.h`](src/ethercat/TurntableController.h) | Turntable servo control -- 120-degree indexing, position verification, mechanical lock interface |
| | [`ScrewDriverController.cpp`](src/ethercat/ScrewDriverController.cpp) / [`.h`](src/ethercat/ScrewDriverController.h) | Screw driver EtherCAT interface -- torque target, angle monitoring, tightening result evaluation |
| | [`SafetyMonitor.cpp`](src/ethercat/SafetyMonitor.cpp) / [`.h`](src/ethercat/SafetyMonitor.h) | FSoE safety communication -- STO commands, safety I/O processing, E-Stop/light curtain integration |
| | [`pybind_module.cpp`](src/ethercat/pybind_module.cpp) | pybind11 Python bindings for C++ EtherCAT modules |
| **Data Types** | [`cell_types.py`](tools/orchestration/cell_types.py) | Enumerations and dataclasses -- product variants, actuator dimensions, screw specs, assembly results |
| | [`system_config.py`](tools/orchestration/system_config.py) | Centralized constants -- station positions, robot coordinates, timing targets, torque thresholds |

---

## Configuration Files

Hardware and system configuration (directory structure prepared, YAML files populated per deployment):

| Directory | Purpose |
|:----------|:--------|
| [`config/codesys/`](config/codesys/) | CODESYS Virtual Control SL container configuration -- task scheduling, cycle times, SoftMotion axes |
| [`config/ethercat/`](config/ethercat/) | EtherCAT network topology -- slave addresses, PDO mappings, DC synchronization parameters |
| [`config/robot/`](config/robot/) | Robot-specific configuration -- TCP offsets, speed limits, assembly positions, torque limits per variant |

---

## Source Material and Design Assumptions

The mechanical and process definition of the station comes from a CloudMinds standardized-workstation solution deck (page renders in [`docs/images/pdf_pages/`](docs/images/pdf_pages/)). The control architecture is this repository's forward-looking design. The split:

| From the source deck | Repository design (not in the deck) |
|:---------------------|:------------------------------------|
| Product variants, part sizes, screw sizes and counts, screw head diameters | Per-step times, daily throughput, OEE targets, turntable motion profile |
| 17-step sequence, work-area duties, station work logic, 50 / 94 / 88 s zone times, 94 s takt | Tightening torques, which feeder holds which screw, camera mounting |
| Equipment list: 1 FANUC + 2 six-axis ROKAE, 3 feeders, 2 drivers, 2 tightening cameras, 1 bit changer, 3 stackers, 15 trays | Robot models, serial numbers, camera models, IP addresses |
| Turntable: 1-1.3 m mother plate, 6 x 3 = 18 fixtures; shared workbench; C-shaped rack layout; combo end-effector | CODESYS Virtual Control SL, IgH EtherCAT, FSoE, OPC UA / MQTT, IPC spec, Beckhoff / Delta / Atlas Copco parts (deck names Siemens SICAR as its software standard) |

Where the deck contradicts itself (step 17 numbering and ownership), the reading adopted here is documented in [Source Interpretation Notes](docs/process-flow.md#source-interpretation-notes).

---

## Technology Stack

| Layer | Components |
|:------|:-----------|
| **Soft PLC** | CODESYS Virtual Control SL (containerized IEC 61131-3), SoftMotion CNC/Robotics, Structured Text / Ladder Logic |
| **Fieldbus** | EtherCAT (1 Gbps, 1 ms cycle), IgH EtherCAT Master (kernel module, PREEMPT_RT 5.15-rt), distributed clock synchronization |
| **Safety** | FSoE (Fail Safe over EtherCAT) SIL3, CODESYS Virtual Safe Control (2026), ISO 13849-1 PLd Category 3 |
| **Robots** | 1x FANUC (material handling, vision + gripper + suction combo tool) + 2x ROKAE (6-axis, screw tightening process robots) |
| **Vision** | 2D cameras: 1 handling camera on the R1 combo end-effector (part localization in trays) + 2 tightening cameras (R2, R3 screw hole alignment), GigE Vision interface |
| **Screw Tightening** | 3x screw feeders (#1 M2.5x4 and #2 M2.5x6 at R2, #3 M2.5x6 at R3) + 2x torque-controlled screw drivers + 1x bit changer |
| **Turntable** | 1-1.3m flexible mother plate, 6 index plates x 3 sub-plates, servo-driven 120-degree indexing |
| **Connectivity** | OPC UA server (MES/SCADA integration), MQTT client (cloud telemetry), digital twin visualization |
| **IPC** | Industrial PC (i7/Xeon, 32GB DDR4, NVMe, dual GbE), PREEMPT_RT Linux kernel 5.15-rt |

---

<div align="center">

**Apache License 2.0** -- See [LICENSE](LICENSE) for details.

</div>
