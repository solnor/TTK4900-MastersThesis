%In this file the functions used in the Matlab System Object are tested.
%In order to better understand and check if they work

%%
%%%%%%%%%%%%%%%%% TESTS %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%5
alt = "/dev/COM5";
alt1 = "/dev/ttyUSB0";
%ser = odrive_connect("/dev/ttyUSB0",115200);
%serialportlist
ser2 = odrive_connect("COM5",115200);
% configureTerminator(ser2,"CR");
% ser2.Timeout = 1;
%% 
autocalibration(ser2)
%% 
run_state(ser2, 0, 2, false)
%%

flush(ser2)
set_velocity(ser2, 0, 0, 0)
odrive_vbus_voltage(ser2);
%%
writeline(ser2, 'f 0\n');
value = readline(ser2)
%odrive_request_state(ser2,0,3)
%% 

%test_1(ser2)
% odrive_read_float(ser2, "axis0.controller.config.control_mode");
% odrive_write_int(ser2, "axis0.controller.config.control_mode",2);
% pause(2);
% odrive_read_float(ser2, "axis0.controller.config.control_mode");
odrive_write_float(ser2, "axis0.controller.input_pos",3)
flush(ser2)
writeline(ser2, 'f 0\n');
value = readline(ser2)
%% 

odrive_read_int(ser2, "axis0.controller.config.control_mode")
odrive_read_int(ser2, "axis0.current_state")

%writeline(ser2, 'v 0 1 0');

%%
odrive_quick_write_int(ser2, 'v', 0, 1)
 %% 
test_4(ser2)

%%
req_state_paramtest_1eter = sprintf("axis%d.current_state", axis);
state = odrive_read_int(ser2, req_state_paramtest_1eter)
%% 

flush(ser2);
odrive_read_int(ser2, 'c 0 0.1\n');
%% 
flush(ser2)
writeline(ser2, 'f 0\n');
value = readline(ser2)
%% 
state = run_state(ser2,0,3,true);
%% 
%%%%%%%%%%%%%%%%%%%%%%%%%%% FUNCTIONS %%%%%%%%%%%%%%%%%%%%

%Connect to the odrive_serial
function serial_odrive = odrive_connect(portname, baudrate)
	serialportObj = serialport(portname,baudrate);
	serialportObj.BaudRate = 115200;
	serialportObj.Timeout = 1;
    serial_odrive = serialportObj;  
end

%Find out if you have to do an int and a float?
function value = odrive_read_float(fd, parameter)
    flush(fd)
	writeline(fd, sprintf('r %s\n', parameter));
	value = readline(fd);
	if isempty(value)
		return
	end
	
	if contains(value, 'invalid property')
		return
	end
	value = str2double(value);
    disp([parameter, "value is ", value]);
	
end

function value = odrive_read_int(fd, parameter)
    flush(fd)
	writeline(fd, sprintf('r %s\n', parameter));
	value = readline(fd);
	if isempty(value)
		return
	end
	
	if contains(value, 'invalid property')
		return
	end
	value = str2double(value);
	
	disp([parameter, "value is ", value]);
	
end

%Odrive write
function output = odrive_write_float(fd, parameter, value)
	flush(fd);
	writeline(fd, sprintf('w %s %f\n', parameter, value));
	output = [];
end

function output = odrive_write_int(fd, parameter, value)
	flush(fd);
	writeline(fd, sprintf('w %s %d\n', parameter, value));
	output = [];
end

function vbus_value = odrive_vbus_voltage(fd)
	vbus_value = odrive_read_int(fd, 'vbus_voltage');
end

%ODrive set velocity

function vel = set_velocity(fd, motor_number, velocity, current_feedforward)
	flush(fd);
	writeline(fd, sprintf('v %i %f %f\n',motor_number, velocity, current_feedforward));
	vel = readline(fd);
end
%Position

function pos = set_position(fd, motor_number, position)
	flush(fd);
	writeline(fd, sprintf('q %i %f\n',motor_number, position));
	pos = readline(fd);
end



function state = run_state(fd, axis, requested_state, wait_for_idle)
    flush(fd);
    odrive_read_int(fd, 'axis0.encoder');
	writeline(fd, sprintf("w axis%d.requested_state %d\n", axis, requested_state));
    disp("Message sent");
    state = 0;
	if wait_for_idle
		pause(100);
		req_state_paramtest_1eter = sprintf("axis%d.current_state", axis)
		state = odrive_read_int(fd, req_state_paramtest_1eter)
	end
	
end

function req_state =  odrive_request_state(fp,  axis, requested_state)
	parameter = sprintf("axis%d.requested_state", axis);
	odrive_write_int(fp, parameter, requested_state);
	req_state = 0;
end


function wait_state = odrive_wait_for_state(fp, axis, requested_state, usleep, n_timeout)
	parameter = sprintf(parameter, "axis%d.current_state", axis);
	state = 0;
	while 1
		state = odrive_read_int(fp, parameter);
		if state == requested_state
			wait_statest_1te = 1;
			break
		end
		if usleep
			pause(usleep/1000);
		end
		if n_timeout == 0
			wait_state=0;
			break
		end
		n_timeout = n_timeout-1;
		
	end
end

function quick_write_int = odrive_quick_write_int(f, type, axis, value)
	writeline(f, sprintf('%c %d %d\n',type, axis, value));
	fprintf(sprintf("WRITE: %c %d %d\n", type, axis, value));
end

function quick_write = odrive_quick_write(f, type, axis, value)
	writeline(f, sprintf('%c %d %f\n',type, axis, value));
	fprintf(sprintf("WRITE: %c %d %f\n", type, axis, value));
end

function autocalib = autocalibration(f)
    fprintf("Calibrating...")
    %Confirmar
    odrive_write_int(f, "axis0.requested_state", 3);
    pause(15)
%     while motor.axis0.motor.is_armed:
%         pass
    autocalib = odrive_read_int(f, "axis0.error");
end
    
%%%%%%%%%%%%%%TESTS%%%%%%%%%%%%%%%%%%%%%%%

function test1 = test_1(f)
	iter = 10;
	parameter = "vbus_voltage";
	
	fprintf(sprintf("Testing %d iterations of reading\n",iter));
	start = tic;
	for i=1:iter
		odrive_read_float(f, parameter);
	end
	elapsed_time = toc(start);
	fprintf(sprintf("Time elapsed: %f ms (per read %f ms)\n", elapsed_time*1000, elapsed_time*1000/iter));
end

function test4 = test_4(f)
	use_index = 1;
	odrive_write_int(f, "axis0.encoder.config.use_index",use_index);
	
	motor_is_calibrated = odrive_read_int(f, "axis0.motor.is_calibrated");
	encoder_is_ready = odrive_read_int(f, "axis0.encoder.is_ready");
	
	fprintf(sprintf("motor is calibrated: %s\n", motor_is_calibrated));
	fprintf(sprintf("encoder is ready: %s\n", encoder_is_ready));
	
	%odrive_write_int(f, "axis0.config.startup_motor_calibration",
	if encoder_is_ready == 1
		fprintf("Trying to encoder get ready\n");
		odrive_write_int(f, "axis0.config.startup_encoder_index_search", 1);
		
		if use_index==1
			print("Using index\n");
			if odrive_read_int(f, "axis0.encoder.config.pre_calibrated")
				odrive_write_int(f, "axis0.config.startup_encoder_offset_calibration", 0);
			else
				odrive_write_int(f, "axis0.config.startup_encoder_offset_calibration", 1);
			end
		else
			odrive_write_int(f, "axis0.config.startup_encoder_offset_calibration", 1);
		end
	else
		odrive_write_int(f, "axis0.config.startup_encoder_index_search", 0);
		odrive_write_int(f, "axis0.config.startup_encoder_offset_calibration", 0);
	end
	
	odrive_request_state(f, 0, 2);
	fprintf("Waiting for get ready\n");
	odrive_wait_for_state(f, 0, 1, 1000000, 0);
end

function test6 = test_6(f)
	fprintf("Contrilling...\n");
	period = 1/50;
	sim_time = 5;
	
	start_time = tic;
	i = 0;
	n = toc(start_time);
	%Escrever um comando de p e medir na sequencia
	odrive_quick_write_int(f, 'p', 0, -2);
	pause(5);
	disp("POS estimate\n");
	disp(odrive_read_float(f, "axis0.encoder.pos_estimate"));
		
	disp("Control done...\n")
end

function test5 = test_5(f)
	fprintf('Reseting errors\n');
	odrive_write_int(f, "axis0.motor.error", 0);
    odrive_write_int(f, "axis0.encoder.error", 0);
    odrive_write_int(f, "axis0.controller.error", 0);
    odrive_write_int(f, "axis0.error", 0);
	
	test_4(f);
	fprintf("Setting current limit to 100A\n");
	odrive_write_float(f, "axis0.motor.config.current_lim", 100);

	fprintf("Setting velocity limit\n");
	odrive_write_float(f, "axis0.controller.config.vel_limit", 100000);

	fprintf("Setting to position mode\n");
	odrive_write_int(f, "axis0.controller.config.control_mode", 3);

	fprintf("Activating regulator\n");
	odrive_request_state(f, 0, 8);
	test6(f, argc, argv);
	odrive_request_state(f, 0, 1);
end
