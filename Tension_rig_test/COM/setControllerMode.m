function setControllerMode(state, device)
% TRENGER IKKE DENNE TROR JEG

% i = -1: Print out possible options


if state == -1
    disp("Controller Modes" + newline)
    disp("0: Voltage Control")
    disp("1: Torque Control ")
    disp("2: Velocity Control ")
    disp("3: Position Control ")
    return
elseif (state < -1) || (state > 3)
    disp("Invalid Number")
    return
else
    command = "w axis0.requested_state " + string(state);
    writeline(device, command)
end

end