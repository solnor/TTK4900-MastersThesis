function setAbsPos(, serialPort)

motor = 0;
command = "c " + string(motor) + " " + string(torque);

% Write Command to Serial Port
write(serialPort, command, 'uint8')


end


