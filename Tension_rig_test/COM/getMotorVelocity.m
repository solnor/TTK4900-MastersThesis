function vel = getMotorVelocity(device)
    command = "r axis0.encoder.vel_estimate\n";
    writeline(device, command);


    % Pause??
    vel = readline(device);
end