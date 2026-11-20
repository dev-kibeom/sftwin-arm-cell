PROGRAM PLC_PRG
VAR
    server : MB.ModbusServer;
    inputWords : ARRAY[0..7] OF WORD;
    holdingWords : ARRAY[0..7] OF WORD;
    pauseLink : BOOL;
    pauseTimer : TON;
END_VAR

// Commissioning only. Words 6 and 7 of the holding block are test controls.
// The PLC publishes a request and echoes its observation of the adapter response.
inputWords[1] := holdingWords[6];
inputWords[2] := holdingWords[1];
inputWords[3] := holdingWords[2];
inputWords[4] := holdingWords[3];
inputWords[5] := holdingWords[4];
inputWords[6] := holdingWords[5];
inputWords[7] := 16#C04E;
IF holdingWords[7] = 1 THEN
    holdingWords[7] := 0;
    pauseLink := TRUE;
END_IF;
pauseTimer(IN := pauseLink, PT := T#3s);
IF pauseTimer.Q THEN
    pauseLink := FALSE;
END_IF;
server(wPort := 1502, InterfaceName := "lo", xEnable := NOT pauseLink,
       pInputData := ADR(inputWords), uiInputDataSize := 8,
       pOutputData := ADR(holdingWords), uiOutputDataSize := 8);
END_PROGRAM
