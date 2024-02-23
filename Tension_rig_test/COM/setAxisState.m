function setAxisState(state, device)
% i = -1: Print out possible options

if state == -1
    disp("Controller Modes" + newline)
    disp("0: Undefined")
    disp("1: Idle ")
    disp("2: Startup Sequence ")
    disp("3: Full Calibration Sequence ")
    disp("4: Motor Calibration ")
    disp("6: Encoder Index Search ")
    disp("7: Encoder Offset Calibration ")
    disp("8: Closed Loop Control ")
    disp("9: Lockin Spin ")
    disp("10: Encoder Dir Find ")
    disp("11: Homing ")
    disp("12: Encoder Hall Polarity Calibration ")
    disp("13: Encoder Hall Phase Calibration " + newline)
    return
elseif (state < -1) || (state > 13)
    disp("Invalid Number")
    return
end

command = "w axis0.requested_state " + string(state);

writeline(device, command)

end