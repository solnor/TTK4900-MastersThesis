function setMotorTorque(torque, device)

motor = 0;
command = "c " + string(motor) + " " + string(torque);

% Write Command to Serial Port
writeline(device, command)

end


