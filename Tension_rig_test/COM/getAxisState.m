function state = getAxisState(device)

command = "r axis0.current_state\n";

writeline(device, command)
% pause????
state = readline(device);
