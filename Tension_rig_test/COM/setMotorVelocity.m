function setMotorVelocity(velocity, torque_ff, serialPort)

motor = 0;
command = "v " + string(motor) + " " + string(velocity) + " " + string(torque_ff);

% Write Command to Serial Port
writeline(serialPort, command)

end


