(* ================================================================
   OP08#-1 Harmonic Reducer Motor Assembly Cell — Main Sequence
   CODESYS Virtual Control SL / IEC 61131-3 Structured Text
   ================================================================
   Main cyclic program (OB1 equivalent).
   Three-zone parallel state machine with turntable interlock,
   vision handshake, screw tightening control, variant switching,
   and production statistics.
   ================================================================ *)

PROGRAM Main_Sequence

VAR
    (* ---- Cell-level state machine ---- *)
    eCellState          : E_CellState := E_CellState.IDLE;
    ePrevCellState      : E_CellState := E_CellState.IDLE;

    (* ---- Zone state machines ---- *)
    eZone1State         : E_ZoneState := E_ZoneState.IDLE;
    eZone2State         : E_ZoneState := E_ZoneState.IDLE;
    eZone3State         : E_ZoneState := E_ZoneState.IDLE;

    (* ---- Zone step counters ---- *)
    nZone1Step          : INT := 0;
    nZone2Step          : INT := 0;
    nZone3Step          : INT := 0;

    (* ---- Product variant ---- *)
    eActiveVariant      : E_ActuatorVariant := E_ActuatorVariant.ACT_8;
    bVariantRequiresBrake : BOOL := FALSE;

    (* ---- HMI recipe commands ---- *)
    bHMI_Start          : BOOL := FALSE;
    bHMI_Stop           : BOOL := FALSE;
    bHMI_Pause          : BOOL := FALSE;
    bHMI_Resume         : BOOL := FALSE;
    bHMI_Reset          : BOOL := FALSE;
    nHMI_VariantSelect  : INT  := 8;         (* 8, 11, or 18 *)

    (* ---- Turntable ---- *)
    bTurntableClamp     : BOOL := FALSE;
    bTurntableUnclamp   : BOOL := FALSE;
    bTurntableIndex     : BOOL := FALSE;
    bTurntableAtPosition: BOOL := FALSE;
    bTurntableClamped   : BOOL := FALSE;
    bTurntableReady     : BOOL := FALSE;
    rTurntablePosition  : REAL := 0.0;

    (* ---- Vision handshake (PLC -> IPC -> result) ---- *)
    bVisionTrigger_R1   : BOOL := FALSE;
    bVisionBusy_R1      : BOOL := FALSE;
    bVisionDone_R1      : BOOL := FALSE;
    bVisionOK_R1        : BOOL := FALSE;
    rVisionDx_R1        : REAL := 0.0;
    rVisionDy_R1        : REAL := 0.0;
    rVisionDrz_R1       : REAL := 0.0;

    (* ---- Screw tightening ---- *)
    bScrewStart_R2      : BOOL := FALSE;
    bScrewBusy_R2       : BOOL := FALSE;
    bScrewDone_R2       : BOOL := FALSE;
    bScrewOK_R2         : BOOL := FALSE;
    nScrewIndex_R2      : INT  := 0;
    rScrewTorque_R2     : REAL := 0.0;
    rScrewAngle_R2      : REAL := 0.0;

    bScrewStart_R3      : BOOL := FALSE;
    bScrewBusy_R3       : BOOL := FALSE;
    bScrewDone_R3       : BOOL := FALSE;
    bScrewOK_R3         : BOOL := FALSE;
    nScrewIndex_R3      : INT  := 0;
    rScrewTorque_R3     : REAL := 0.0;
    rScrewAngle_R3      : REAL := 0.0;

    (* ---- Robot I/O signals ---- *)
    bR1_InPosition      : BOOL := FALSE;
    bR1_GripperClosed   : BOOL := FALSE;
    bR1_SuctionON       : BOOL := FALSE;
    bR2_InPosition      : BOOL := FALSE;
    bR2_GripperClosed   : BOOL := FALSE;
    bR3_InPosition      : BOOL := FALSE;
    bR3_GripperClosed   : BOOL := FALSE;

    bR1_MoveCmd         : BOOL := FALSE;
    bR2_MoveCmd         : BOOL := FALSE;
    bR3_MoveCmd         : BOOL := FALSE;

    (* ---- Safety ---- *)
    bSafetyOK           : BOOL := TRUE;
    bEStopActive        : BOOL := FALSE;

    (* ---- Production statistics ---- *)
    nCycleCount         : UDINT := 0;
    nGoodCount          : UDINT := 0;
    nBadCount           : UDINT := 0;
    tCycleTimer         : TON;
    tCycleDuration_ms   : UDINT := 0;
    rTaktTarget_ms      : REAL := 94000.0;   (* 94 s *)
    rOEE                : REAL := 0.0;

    (* ---- Internal flags ---- *)
    bAllZonesDone       : BOOL := FALSE;
    bCycleComplete      : BOOL := FALSE;
    bFirstScan          : BOOL := TRUE;

    (* ---- Turntable handshake (PLC <-> C++ controller) ---- *)
    nTurntableSeqState  : INT := 0;
    bTurntableUnclampCmd: BOOL := FALSE;
    bTurntableUnclampAck: BOOL := FALSE;
    bTurntableIndexCmd  : BOOL := FALSE;
    bTurntableIndexAck  : BOOL := FALSE;
    bTurntableClampedAck: BOOL := FALSE;
    TON_TurntableTimeout: TON;
END_VAR

(* ================================================================
   TYPE DEFINITIONS
   ================================================================ *)

TYPE E_CellState : (
    IDLE   := 0,
    HOMING := 1,
    READY  := 2,
    RUNNING:= 3,
    PAUSED := 4,
    ERROR  := 5,
    ESTOP  := 6
);
END_TYPE

TYPE E_ZoneState : (
    IDLE    := 0,
    BUSY    := 10,
    DONE    := 20,
    ERROR   := 90,
    WAITING := 30
);
END_TYPE

TYPE E_ActuatorVariant : (
    ACT_8  := 8,
    ACT_11 := 11,
    ACT_18 := 18
);
END_TYPE

(* ================================================================
   MAIN BODY — executed every PLC cycle (1 ms)
   ================================================================ *)

(* ---- First-scan initialisation ---- *)
IF bFirstScan THEN
    eCellState := E_CellState.IDLE;
    bFirstScan := FALSE;
END_IF

(* ---- Emergency stop override ---- *)
IF bEStopActive OR NOT bSafetyOK THEN
    eCellState := E_CellState.ESTOP;
    bR1_MoveCmd := FALSE;
    bR2_MoveCmd := FALSE;
    bR3_MoveCmd := FALSE;
    bTurntableIndex := FALSE;
    bScrewStart_R2 := FALSE;
    bScrewStart_R3 := FALSE;
END_IF

(* ---- HMI variant selection ---- *)
CASE nHMI_VariantSelect OF
    8:  eActiveVariant := E_ActuatorVariant.ACT_8;
        bVariantRequiresBrake := FALSE;
    11: eActiveVariant := E_ActuatorVariant.ACT_11;
        bVariantRequiresBrake := TRUE;
    18: eActiveVariant := E_ActuatorVariant.ACT_18;
        bVariantRequiresBrake := TRUE;
END_CASE

(* ================================================================
   CELL STATE MACHINE
   ================================================================ *)
CASE eCellState OF

    (* ---- IDLE -------------------------------------------------- *)
    E_CellState.IDLE:
        IF bHMI_Start THEN
            bHMI_Start := FALSE;
            eCellState := E_CellState.HOMING;
        END_IF

    (* ---- HOMING ------------------------------------------------ *)
    E_CellState.HOMING:
        bR1_MoveCmd := TRUE;   (* send home commands *)
        bR2_MoveCmd := TRUE;
        bR3_MoveCmd := TRUE;
        IF bR1_InPosition AND bR2_InPosition AND bR3_InPosition THEN
            bR1_MoveCmd := FALSE;
            bR2_MoveCmd := FALSE;
            bR3_MoveCmd := FALSE;
            bTurntableClamp := TRUE;
            eCellState := E_CellState.READY;
        END_IF

    (* ---- READY ------------------------------------------------- *)
    E_CellState.READY:
        IF bHMI_Start THEN
            bHMI_Start := FALSE;
            nCycleCount := 0;
            nGoodCount  := 0;
            nBadCount   := 0;
            eCellState  := E_CellState.RUNNING;
        END_IF

    (* ---- RUNNING ----------------------------------------------- *)
    E_CellState.RUNNING:
        (* Pause request *)
        IF bHMI_Pause THEN
            bHMI_Pause := FALSE;
            ePrevCellState := eCellState;
            eCellState := E_CellState.PAUSED;
        END_IF

        (* Stop request *)
        IF bHMI_Stop THEN
            bHMI_Stop := FALSE;
            eCellState := E_CellState.READY;
        END_IF

        (* ----- Run the three-zone parallel state machine ----- *)

        (* Start cycle timer *)
        tCycleTimer(IN := TRUE, PT := T#200S);

        (* --- Zone 1: R1 Material Handling --- *)
        CASE eZone1State OF
            E_ZoneState.IDLE:
                nZone1Step := 0;
                eZone1State := E_ZoneState.BUSY;

            E_ZoneState.BUSY:
                CASE nZone1Step OF
                    0: (* Trigger vision for part localisation *)
                        bVisionTrigger_R1 := TRUE;
                        IF bVisionDone_R1 AND bVisionOK_R1 THEN
                            bVisionTrigger_R1 := FALSE;
                            nZone1Step := 1;
                        ELSIF bVisionDone_R1 AND NOT bVisionOK_R1 THEN
                            bVisionTrigger_R1 := FALSE;
                            eZone1State := E_ZoneState.ERROR;
                        END_IF

                    1: (* Pick motor mount *)
                        bR1_MoveCmd := TRUE;
                        IF bR1_InPosition THEN
                            bR1_MoveCmd := FALSE;
                            bR1_GripperClosed := TRUE;
                            nZone1Step := 2;
                        END_IF

                    2: (* Place motor mount on turntable fixture *)
                        bR1_MoveCmd := TRUE;
                        IF bR1_InPosition THEN
                            bR1_MoveCmd := FALSE;
                            bR1_GripperClosed := FALSE;
                            nZone1Step := 3;
                        END_IF

                    3: (* Pick motor and place *)
                        bR1_SuctionON := TRUE;
                        bR1_MoveCmd := TRUE;
                        IF bR1_InPosition THEN
                            bR1_MoveCmd := FALSE;
                            bR1_SuctionON := FALSE;
                            IF bVariantRequiresBrake THEN
                                nZone1Step := 4;
                            ELSE
                                eZone1State := E_ZoneState.DONE;
                            END_IF
                        END_IF

                    4: (* Pick brake magnet base — only #11 & #18 *)
                        bR1_MoveCmd := TRUE;
                        IF bR1_InPosition THEN
                            bR1_MoveCmd := FALSE;
                            eZone1State := E_ZoneState.DONE;
                        END_IF
                END_CASE

            E_ZoneState.DONE:
                ; (* wait for turntable sync *)

            E_ZoneState.ERROR:
                eCellState := E_CellState.ERROR;
        END_CASE

        (* --- Zone 2: R2 Main Assembly (motor mount + motor + 4 screws) --- *)
        CASE eZone2State OF
            E_ZoneState.IDLE:
                nZone2Step := 0;
                eZone2State := E_ZoneState.BUSY;

            E_ZoneState.BUSY:
                CASE nZone2Step OF
                    0: (* Locate motor mount on fixture *)
                        bR2_MoveCmd := TRUE;
                        IF bR2_InPosition THEN
                            bR2_MoveCmd := FALSE;
                            nZone2Step := 1;
                        END_IF

                    1: (* Pick up motor mount and seat on assembly jig *)
                        bR2_GripperClosed := TRUE;
                        bR2_MoveCmd := TRUE;
                        IF bR2_InPosition THEN
                            bR2_MoveCmd := FALSE;
                            bR2_GripperClosed := FALSE;
                            nZone2Step := 2;
                        END_IF

                    2: (* Pick motor and insert into mount *)
                        bR2_GripperClosed := TRUE;
                        bR2_MoveCmd := TRUE;
                        IF bR2_InPosition THEN
                            bR2_MoveCmd := FALSE;
                            bR2_GripperClosed := FALSE;
                            nZone2Step := 3;
                        END_IF

                    3: (* Screw 1 of 4 — M2.5x4 *)
                        nScrewIndex_R2 := 1;
                        bScrewStart_R2 := TRUE;
                        IF bScrewDone_R2 THEN
                            bScrewStart_R2 := FALSE;
                            IF bScrewOK_R2 THEN
                                nZone2Step := 4;
                            ELSE
                                eZone2State := E_ZoneState.ERROR;
                            END_IF
                        END_IF

                    4: (* Screw 2 of 4 *)
                        nScrewIndex_R2 := 2;
                        bScrewStart_R2 := TRUE;
                        IF bScrewDone_R2 THEN
                            bScrewStart_R2 := FALSE;
                            IF bScrewOK_R2 THEN
                                nZone2Step := 5;
                            ELSE
                                eZone2State := E_ZoneState.ERROR;
                            END_IF
                        END_IF

                    5: (* Screw 3 of 4 *)
                        nScrewIndex_R2 := 3;
                        bScrewStart_R2 := TRUE;
                        IF bScrewDone_R2 THEN
                            bScrewStart_R2 := FALSE;
                            IF bScrewOK_R2 THEN
                                nZone2Step := 6;
                            ELSE
                                eZone2State := E_ZoneState.ERROR;
                            END_IF
                        END_IF

                    6: (* Screw 4 of 4 *)
                        nScrewIndex_R2 := 4;
                        bScrewStart_R2 := TRUE;
                        IF bScrewDone_R2 THEN
                            bScrewStart_R2 := FALSE;
                            IF bScrewOK_R2 THEN
                                eZone2State := E_ZoneState.DONE;
                            ELSE
                                eZone2State := E_ZoneState.ERROR;
                            END_IF
                        END_IF
                END_CASE

            E_ZoneState.DONE:
                ;

            E_ZoneState.ERROR:
                eCellState := E_CellState.ERROR;
        END_CASE

        (* --- Zone 3: R3 Brake Magnet Base (2 screws M2.5x6) --- *)
        CASE eZone3State OF
            E_ZoneState.IDLE:
                IF bVariantRequiresBrake THEN
                    nZone3Step := 0;
                    eZone3State := E_ZoneState.BUSY;
                ELSE
                    eZone3State := E_ZoneState.DONE;  (* skip for #8 *)
                END_IF

            E_ZoneState.BUSY:
                CASE nZone3Step OF
                    0: (* Pick brake magnet base from fixture *)
                        bR3_GripperClosed := TRUE;
                        bR3_MoveCmd := TRUE;
                        IF bR3_InPosition THEN
                            bR3_MoveCmd := FALSE;
                            nZone3Step := 1;
                        END_IF

                    1: (* Screw 1 of 2 — M2.5x6 *)
                        nScrewIndex_R3 := 1;
                        bScrewStart_R3 := TRUE;
                        IF bScrewDone_R3 THEN
                            bScrewStart_R3 := FALSE;
                            IF bScrewOK_R3 THEN
                                nZone3Step := 2;
                            ELSE
                                eZone3State := E_ZoneState.ERROR;
                            END_IF
                        END_IF

                    2: (* Screw 2 of 2 *)
                        nScrewIndex_R3 := 2;
                        bScrewStart_R3 := TRUE;
                        IF bScrewDone_R3 THEN
                            bScrewStart_R3 := FALSE;
                            IF bScrewOK_R3 THEN
                                eZone3State := E_ZoneState.DONE;
                            ELSE
                                eZone3State := E_ZoneState.ERROR;
                            END_IF
                        END_IF
                END_CASE

            E_ZoneState.DONE:
                ;

            E_ZoneState.ERROR:
                eCellState := E_CellState.ERROR;
        END_CASE

        (* ----- Turntable interlock and rotation (state-based handshake) ----- *)
        bAllZonesDone := (eZone1State = E_ZoneState.DONE)
                     AND (eZone2State = E_ZoneState.DONE)
                     AND (eZone3State = E_ZoneState.DONE);

        (* Turntable index sequence with explicit handshake *)
        CASE nTurntableSeqState OF
            0: (* Wait for all zones done *)
                IF bAllZonesDone AND NOT bCycleComplete THEN
                    bTurntableUnclampCmd := TRUE;
                    nTurntableSeqState := 1;
                    TON_TurntableTimeout(IN := FALSE);
                END_IF;

            1: (* Wait for unclamp acknowledgement from C++ controller *)
                IF bTurntableUnclampAck THEN
                    bTurntableUnclampCmd := FALSE;
                    nTurntableSeqState := 2;
                    TON_TurntableTimeout(IN := FALSE);
                END_IF;
                (* Timeout watchdog *)
                TON_TurntableTimeout(IN := TRUE, PT := T#5s);
                IF TON_TurntableTimeout.Q THEN
                    nTurntableSeqState := 99; (* Error *)
                END_IF;

            2: (* Command index rotation *)
                bTurntableIndexCmd := TRUE;
                nTurntableSeqState := 3;
                TON_TurntableTimeout(IN := FALSE);

            3: (* Wait for index complete acknowledgement *)
                IF bTurntableIndexAck THEN
                    bTurntableIndexCmd := FALSE;
                    nTurntableSeqState := 4;
                    TON_TurntableTimeout(IN := FALSE);
                END_IF;
                (* Timeout watchdog *)
                TON_TurntableTimeout(IN := TRUE, PT := T#5s);
                IF TON_TurntableTimeout.Q THEN
                    nTurntableSeqState := 99; (* Error *)
                END_IF;

            4: (* Wait for clamp confirmation *)
                IF bTurntableClampedAck THEN
                    nTurntableSeqState := 0; (* Ready for next cycle *)
                    bTurntableReady := TRUE;
                    bCycleComplete := TRUE;

                    (* Update statistics *)
                    tCycleTimer(IN := FALSE);
                    tCycleDuration_ms := TIME_TO_UDINT(tCycleTimer.ET);
                    nCycleCount := nCycleCount + 1;
                    nGoodCount  := nGoodCount + 1;

                    (* Simple OEE calculation *)
                    IF nCycleCount > 0 THEN
                        rOEE := UDINT_TO_REAL(nGoodCount)
                              / UDINT_TO_REAL(nCycleCount);
                    END_IF
                END_IF;

            99: (* Error state *)
                bTurntableUnclampCmd := FALSE;
                bTurntableIndexCmd := FALSE;
                eCellState := E_CellState.ERROR;
        END_CASE;

        (* Reset for next cycle *)
        IF bCycleComplete THEN
            eZone1State := E_ZoneState.IDLE;
            eZone2State := E_ZoneState.IDLE;
            eZone3State := E_ZoneState.IDLE;
            bCycleComplete := FALSE;
            bTurntableReady := FALSE;
        END_IF

    (* ---- PAUSED ------------------------------------------------ *)
    E_CellState.PAUSED:
        bR1_MoveCmd := FALSE;
        bR2_MoveCmd := FALSE;
        bR3_MoveCmd := FALSE;
        IF bHMI_Resume THEN
            bHMI_Resume := FALSE;
            eCellState := ePrevCellState;
        END_IF

    (* ---- ERROR ------------------------------------------------- *)
    E_CellState.ERROR:
        bR1_MoveCmd := FALSE;
        bR2_MoveCmd := FALSE;
        bR3_MoveCmd := FALSE;
        bScrewStart_R2 := FALSE;
        bScrewStart_R3 := FALSE;
        IF bHMI_Reset THEN
            bHMI_Reset := FALSE;
            eZone1State := E_ZoneState.IDLE;
            eZone2State := E_ZoneState.IDLE;
            eZone3State := E_ZoneState.IDLE;
            nZone1Step := 0;
            nZone2Step := 0;
            nZone3Step := 0;
            eCellState := E_CellState.IDLE;
        END_IF

    (* ---- ESTOP ------------------------------------------------- *)
    E_CellState.ESTOP:
        (* All outputs forced OFF by safety_program *)
        IF NOT bEStopActive AND bSafetyOK THEN
            IF bHMI_Reset THEN
                bHMI_Reset := FALSE;
                eCellState := E_CellState.IDLE;
            END_IF
        END_IF

END_CASE

END_PROGRAM
