%% Port info
port = "COM4";
baudrate = 115200;



%% Test
str = "r vbus_voltage ";
C = char(str);
command = double(C)

% sim("sim_test.slx")