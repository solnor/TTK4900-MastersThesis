clear
baudrate = 115200;
driver1 = serialport("COM4", baudrate, "Timeout",0.5);
%%
writeline(driver1,"r vbus_voltage")
readline(driver1)
% setAxisState(8,driver1);
% setMotorTorque(-1,driver1)
% pause(10)
% setMotorTorque(0,driver1)
% pause(5)
% setAxisState(1,driver1);
clear