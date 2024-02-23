function getFeedback(serialPort)

motor = 0;
command = "f " + string(motor);

% Write Command to Serial Port
writeline(serialPort, command)

end


