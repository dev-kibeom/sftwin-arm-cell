PROGRAM PLC_PRG
VAR RETAIN
    phase : WORD := 0;
    cycle : DWORD := 0;
    requested : DWORD := 0;
    committed : DWORD := 0;
    resultState, resultReason, placement : WORD;
END_VAR
VAR
    server : MB.ModbusServer;
    ir : ARRAY[0..31] OF WORD;
    hr : ARRAY[0..25] OF WORD;
    lastEquipment, lastReply, lastReady : WORD;
    seenEquipment, seenReply, seenReady : BOOL;
    equipmentAge, replyAge, readyAge : TON;
    valid, link, robotReady, automatic, permitted, motion : BOOL;
    infeed, pick, stopped, extended, occupied, pathClear : BOOL;
    driveAvailable, pickZoneClear, dischargeZoneClear, onBelt : BOOL;
    feed, runBelt, extend, discharge, confirmed : BOOL;
    generated, boundSlot, replyId : DWORD;
END_VAR

IF hr[16] <> lastEquipment THEN seenEquipment := TRUE; END_IF;
IF hr[5] <> lastReply THEN seenReply := TRUE; END_IF;
IF hr[6] <> lastReady THEN seenReady := TRUE; END_IF;
equipmentAge(IN := hr[16] = lastEquipment, PT := T#1s);
replyAge(IN := hr[5] = lastReply, PT := T#1s);
readyAge(IN := hr[6] = lastReady, PT := T#1s);
lastEquipment := hr[16];
lastReply := hr[5];
lastReady := hr[6];
valid := seenEquipment AND NOT equipmentAge.Q AND hr[17] = 1;
link := seenReply AND NOT replyAge.Q;
robotReady := link AND seenReady AND NOT readyAge.Q AND hr[7] = 1;
automatic := hr[18] = 1;
infeed := hr[21].0;
pick := hr[21].1;
stopped := hr[21].2;
extended := hr[21].3;
occupied := hr[21].4;
pathClear := hr[21].5;
driveAvailable := hr[21].6;
pickZoneClear := hr[21].7;
dischargeZoneClear := hr[21].8;
permitted := hr[21].9;
onBelt := hr[21].10;
generated := SHL(WORD_TO_DWORD(hr[22]), 16) OR hr[23];
boundSlot := SHL(WORD_TO_DWORD(hr[24]), 16) OR hr[25];
replyId := SHL(WORD_TO_DWORD(hr[0]), 16) OR hr[1];
motion := valid AND automatic AND permitted;
feed := FALSE;
runBelt := FALSE;
extend := FALSE;
discharge := FALSE;
confirmed := FALSE;

CASE phase OF
0:  // P1: reconcile both owners before allocating a new material identity.
    IF motion AND robotReady AND hr[19] = 1 AND hr[20] > 0 AND
       generated = cycle AND committed = cycle AND requested = cycle AND
       replyId = requested AND (hr[2] = 0 OR (hr[2] >= 4 AND hr[2] <= 6)) AND
       NOT infeed AND NOT pick AND NOT occupied AND pathClear AND NOT onBelt AND
       stopped AND pickZoneClear AND cycle < 16#FFFFFFFF THEN
        cycle := cycle + 1;
        phase := 10;
    END_IF;
10: // Hold the same feed ID until actual creation and infeed detection.
    IF valid AND generated = cycle AND infeed THEN
        phase := 20;
    ELSIF motion AND robotReady AND generated = cycle - 1 AND NOT infeed AND NOT pick AND
          NOT occupied AND pathClear AND NOT onBelt AND stopped AND pickZoneClear AND
          hr[19] = 1 AND hr[20] > 0 THEN
        feed := TRUE;
    END_IF;
20: // Arm arrival detection only when this transfer starts with the sensor clear.
    IF motion AND driveAvailable AND pickZoneClear AND NOT pick AND infeed AND
       NOT occupied AND generated = cycle THEN
        runBelt := TRUE;
        phase := 21;
    END_IF;
21: // Drive Busy pauses this transfer. Never restart an unlocated part.
    IF valid AND pick AND generated = cycle THEN
        phase := 22;
    ELSIF motion AND driveAvailable AND pickZoneClear AND NOT occupied AND
          generated = cycle AND (infeed OR onBelt) THEN
        runBelt := TRUE;
    END_IF;
22: // Stop, fix, then expose the pick confirmation until the provider binds its slot.
    extend := motion AND pick AND stopped AND pickZoneClear AND generated = cycle;
    confirmed := extend AND extended AND NOT occupied;
    IF confirmed AND boundSlot = cycle AND robotReady THEN
        requested := cycle;
        resultState := 0;
        resultReason := 0;
        placement := 0;
        phase := 30;
    END_IF;
30: // Keep the request ID visible while the existing Adapter resolves this job.
    extend := motion AND stopped AND extended;
    IF link AND replyId = requested AND hr[2] >= 4 AND hr[2] <= 7 THEN
        resultState := hr[2];
        resultReason := hr[3];
        placement := hr[4];
        phase := 40;
    END_IF;
40: // P4 is bookkeeping, including when the robot failed after Place.
    IF valid AND occupied AND placement = 2 AND resultState <= 6 AND committed <> cycle THEN
        committed := cycle;
        phase := 50;
    END_IF;
50: // Do not credit an empty slot until a permitted discharge was commanded.
    IF motion AND occupied AND dischargeZoneClear THEN
        discharge := TRUE;
        phase := 51;
    END_IF;
51:
    discharge := motion AND occupied AND dischargeZoneClear;
    IF motion AND NOT occupied AND NOT infeed AND NOT pick AND pathClear AND
       NOT onBelt AND stopped AND pickZoneClear AND robotReady THEN
        phase := 0;
    END_IF;
END_CASE;

ir[0] := DWORD_TO_WORD(SHR(requested, 16));
ir[1] := DWORD_TO_WORD(requested);
ir[16] := DWORD_TO_WORD(SHR(cycle, 16));
ir[17] := DWORD_TO_WORD(cycle);
ir[18] := phase;
ir[19] := BOOL_TO_WORD(feed);
ir[20] := BOOL_TO_WORD(runBelt);
ir[21] := BOOL_TO_WORD(extend);
ir[22] := BOOL_TO_WORD(confirmed);
ir[23] := BOOL_TO_WORD(discharge);
ir[24] := DWORD_TO_WORD(SHR(committed, 16));
ir[25] := DWORD_TO_WORD(committed);
ir[26] := resultState;
ir[27] := resultReason;
ir[28] := placement;
ir[29] := 16#C051;
// Publish the retained request before opening the server on the first scan.
// FC04: PLC-owned commands. FC16: one writer per input block (see README).
server(wPort := 1502, InterfaceName := "lo", xEnable := TRUE,
       pInputData := ADR(ir), uiInputDataSize := 32,
       pOutputData := ADR(hr), uiOutputDataSize := 26);
END_PROGRAM
