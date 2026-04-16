(* ================================================================
   OP08#-1 Harmonic Reducer Motor Assembly Cell — Safety Program
   CODESYS Virtual Control SL / IEC 61131-3 Structured Text
   ================================================================
   Safety PLC — ISO 13849-1 PLd Category 3
   FSoE protocol handling, E-Stop, light curtain, safety door,
   STO / SLS for robot drives, conveyor muting logic.
   ================================================================ *)

PROGRAM Safety_Program

VAR
    (* ================================================================
       SAFETY INPUTS (directly from EL1904 safety DI modules)
       ================================================================ *)

    (* E-Stop buttons — dual-channel per button *)
    bEStop1_Ch1         : BOOL := FALSE;   (* Panel front *)
    bEStop1_Ch2         : BOOL := FALSE;
    bEStop2_Ch1         : BOOL := FALSE;   (* Panel rear *)
    bEStop2_Ch2         : BOOL := FALSE;
    bEStop3_Ch1         : BOOL := FALSE;   (* Teach pendant *)
    bEStop3_Ch2         : BOOL := FALSE;

    (* Light curtains — dual-channel *)
    bLightCurtain1_Ch1  : BOOL := FALSE;   (* Zone 1 entry *)
    bLightCurtain1_Ch2  : BOOL := FALSE;
    bLightCurtain2_Ch1  : BOOL := FALSE;   (* Zone 2/3 entry *)
    bLightCurtain2_Ch2  : BOOL := FALSE;

    (* Safety doors — dual-channel per door *)
    bSafetyDoor1_Ch1    : BOOL := FALSE;   (* Maintenance door left *)
    bSafetyDoor1_Ch2    : BOOL := FALSE;
    bSafetyDoor2_Ch1    : BOOL := FALSE;   (* Maintenance door right *)
    bSafetyDoor2_Ch2    : BOOL := FALSE;

    (* Conveyor muting sensors *)
    bMuteSensor1_Entry  : BOOL := FALSE;   (* Upstream conveyor mute *)
    bMuteSensor1_Exit   : BOOL := FALSE;
    bMuteSensor2_Entry  : BOOL := FALSE;   (* Downstream conveyor mute *)
    bMuteSensor2_Exit   : BOOL := FALSE;

    (* ================================================================
       SAFETY OUTPUTS (to EL2904 safety DO modules)
       ================================================================ *)

    (* Safe Torque Off for robot drives *)
    bSTO_R1             : BOOL := FALSE;   (* FANUC R1 *)
    bSTO_R2             : BOOL := FALSE;   (* ROKAE R2 *)
    bSTO_R3             : BOOL := FALSE;   (* ROKAE R3 *)
    bSTO_Turntable      : BOOL := FALSE;   (* Turntable servo *)

    (* Safely Limited Speed *)
    bSLS_R1             : BOOL := FALSE;   (* Reduced speed mode R1 *)
    bSLS_R2             : BOOL := FALSE;
    bSLS_R3             : BOOL := FALSE;

    (* Safety relay outputs *)
    bSafetyRelay_Main   : BOOL := FALSE;   (* Main contactor *)
    bSafetyRelay_Pneum  : BOOL := FALSE;   (* Pneumatic supply *)

    (* Warning lamps *)
    bLamp_Red           : BOOL := FALSE;
    bLamp_Yellow        : BOOL := FALSE;
    bLamp_Green         : BOOL := FALSE;

    (* ================================================================
       INTERNAL VARIABLES
       ================================================================ *)

    (* Aggregated safety flags *)
    bEStop_Active       : BOOL := FALSE;
    bLightCurtain_OK    : BOOL := FALSE;
    bDoors_Closed       : BOOL := FALSE;
    bSafety_OK          : BOOL := FALSE;

    (* Discrepancy timers — Cat 3 cross-channel monitoring *)
    tDiscrepEStop1      : TON;
    tDiscrepEStop2      : TON;
    tDiscrepEStop3      : TON;
    tDiscrepLC1         : TON;
    tDiscrepLC2         : TON;
    tDiscrepDoor1       : TON;
    tDiscrepDoor2       : TON;
    T_DISCREP_MAX       : TIME := T#500MS;

    bDiscrepFault       : BOOL := FALSE;

    (* Muting state *)
    bMuteActive_LC1     : BOOL := FALSE;
    bMuteActive_LC2     : BOOL := FALSE;
    tMuteTimeout1       : TON;
    tMuteTimeout2       : TON;
    T_MUTE_MAX          : TIME := T#10S;

    (* Reset *)
    bResetButton        : BOOL := FALSE;
    bResetRequired      : BOOL := FALSE;
END_VAR


(* ================================================================
   1. DISCREPANCY MONITORING (Category 3)
   Both channels of each safety device must agree within T_DISCREP_MAX.
   ================================================================ *)

(* E-Stop 1 *)
tDiscrepEStop1(IN := (bEStop1_Ch1 XOR bEStop1_Ch2), PT := T_DISCREP_MAX);
(* E-Stop 2 *)
tDiscrepEStop2(IN := (bEStop2_Ch1 XOR bEStop2_Ch2), PT := T_DISCREP_MAX);
(* E-Stop 3 *)
tDiscrepEStop3(IN := (bEStop3_Ch1 XOR bEStop3_Ch2), PT := T_DISCREP_MAX);
(* Light curtain 1 *)
tDiscrepLC1(IN := (bLightCurtain1_Ch1 XOR bLightCurtain1_Ch2), PT := T_DISCREP_MAX);
(* Light curtain 2 *)
tDiscrepLC2(IN := (bLightCurtain2_Ch1 XOR bLightCurtain2_Ch2), PT := T_DISCREP_MAX);
(* Safety door 1 *)
tDiscrepDoor1(IN := (bSafetyDoor1_Ch1 XOR bSafetyDoor1_Ch2), PT := T_DISCREP_MAX);
(* Safety door 2 *)
tDiscrepDoor2(IN := (bSafetyDoor2_Ch1 XOR bSafetyDoor2_Ch2), PT := T_DISCREP_MAX);

bDiscrepFault := tDiscrepEStop1.Q OR tDiscrepEStop2.Q OR tDiscrepEStop3.Q
              OR tDiscrepLC1.Q    OR tDiscrepLC2.Q
              OR tDiscrepDoor1.Q  OR tDiscrepDoor2.Q;


(* ================================================================
   2. E-STOP EVALUATION
   Active-low (NC contacts): TRUE = button released / safe
   ================================================================ *)

bEStop_Active := NOT (bEStop1_Ch1 AND bEStop1_Ch2)
              OR NOT (bEStop2_Ch1 AND bEStop2_Ch2)
              OR NOT (bEStop3_Ch1 AND bEStop3_Ch2);

IF bEStop_Active THEN
    bResetRequired := TRUE;
END_IF


(* ================================================================
   3. LIGHT CURTAIN EVALUATION (with muting)
   ================================================================ *)

(* --- Muting logic for conveyor zone 1 --- *)
(* Mute activates when both mute sensors see a part simultaneously *)
IF bMuteSensor1_Entry AND bMuteSensor1_Exit THEN
    bMuteActive_LC1 := TRUE;
ELSIF NOT bMuteSensor1_Entry AND NOT bMuteSensor1_Exit THEN
    bMuteActive_LC1 := FALSE;
END_IF

(* Muting timeout — safety: cannot stay muted indefinitely *)
tMuteTimeout1(IN := bMuteActive_LC1, PT := T_MUTE_MAX);
IF tMuteTimeout1.Q THEN
    bMuteActive_LC1 := FALSE;   (* force un-mute on timeout *)
END_IF

(* --- Muting logic for conveyor zone 2 --- *)
IF bMuteSensor2_Entry AND bMuteSensor2_Exit THEN
    bMuteActive_LC2 := TRUE;
ELSIF NOT bMuteSensor2_Entry AND NOT bMuteSensor2_Exit THEN
    bMuteActive_LC2 := FALSE;
END_IF

tMuteTimeout2(IN := bMuteActive_LC2, PT := T_MUTE_MAX);
IF tMuteTimeout2.Q THEN
    bMuteActive_LC2 := FALSE;
END_IF

(* Effective light curtain status *)
bLightCurtain_OK := ((bLightCurtain1_Ch1 AND bLightCurtain1_Ch2) OR bMuteActive_LC1)
                AND ((bLightCurtain2_Ch1 AND bLightCurtain2_Ch2) OR bMuteActive_LC2);


(* ================================================================
   4. SAFETY DOOR EVALUATION
   ================================================================ *)

bDoors_Closed := (bSafetyDoor1_Ch1 AND bSafetyDoor1_Ch2)
             AND (bSafetyDoor2_Ch1 AND bSafetyDoor2_Ch2);


(* ================================================================
   5. AGGREGATE SAFETY STATUS
   ================================================================ *)

bSafety_OK := NOT bEStop_Active
          AND bLightCurtain_OK
          AND bDoors_Closed
          AND NOT bDiscrepFault
          AND NOT bResetRequired;


(* ================================================================
   6. STO / SLS OUTPUT LOGIC
   ================================================================ *)

IF bSafety_OK THEN
    (* All clear — enable drives *)
    bSTO_R1         := TRUE;
    bSTO_R2         := TRUE;
    bSTO_R3         := TRUE;
    bSTO_Turntable  := TRUE;
    bSLS_R1         := FALSE;
    bSLS_R2         := FALSE;
    bSLS_R3         := FALSE;

    bSafetyRelay_Main  := TRUE;
    bSafetyRelay_Pneum := TRUE;

ELSIF NOT bEStop_Active AND NOT bDoors_Closed AND bLightCurtain_OK THEN
    (* Door open but no E-Stop — reduced speed mode *)
    bSTO_R1         := TRUE;
    bSTO_R2         := TRUE;
    bSTO_R3         := TRUE;
    bSTO_Turntable  := FALSE;   (* turntable locked *)
    bSLS_R1         := TRUE;    (* 250 mm/s max *)
    bSLS_R2         := TRUE;
    bSLS_R3         := TRUE;

    bSafetyRelay_Main  := TRUE;
    bSafetyRelay_Pneum := TRUE;

ELSE
    (* Full safety stop *)
    bSTO_R1         := FALSE;
    bSTO_R2         := FALSE;
    bSTO_R3         := FALSE;
    bSTO_Turntable  := FALSE;
    bSLS_R1         := FALSE;
    bSLS_R2         := FALSE;
    bSLS_R3         := FALSE;

    bSafetyRelay_Main  := FALSE;
    bSafetyRelay_Pneum := FALSE;
END_IF


(* ================================================================
   7. SIGNAL LAMPS
   ================================================================ *)

bLamp_Green  := bSafety_OK;
bLamp_Yellow := NOT bSafety_OK AND NOT bEStop_Active;
bLamp_Red    := bEStop_Active OR bDiscrepFault;


(* ================================================================
   8. MANUAL RESET ACKNOWLEDGEMENT
   ================================================================ *)

IF bResetRequired AND bResetButton THEN
    IF NOT bEStop_Active AND bDoors_Closed AND bLightCurtain_OK
       AND NOT bDiscrepFault THEN
        bResetRequired := FALSE;
    END_IF
END_IF

END_PROGRAM
