function pos = getMotorPosition(device)
    command = "r axis0.encoder.pos_estimate\n";
    writeline(device, command);


    % Pause??
    pos = readline(device);
end