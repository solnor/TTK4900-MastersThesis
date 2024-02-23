classdef (StrictDefaults) system_odrive_serial< matlab.System
    % This is a system blcok to measure the different aspects of life
    

    properties

    end

    properties (Nontunable)
        Autocalibration = 'Disabled (fail if not calibrated)'
    end

%     properties(Nontunable, PositiveInteger)
%         Baudrate = 115200
%     end

    properties (Nontunable, Logical)
        EnableAxis0 = true; % Enable
        EnableAxis1 = false; % Enable
        
        EnableVbusOutput = false; % Bus voltage output
        EnableTiming = false; % Enable block timing

        EnablePosition0Output = false; % Enable estimated position output
        EnableVelocity0Output = false; % Enable estimated velocity output
        EnableCurrent0Output = false; % Enable estimated current output

        EnablePosition1Output = false; % Enable estimated position output

        UseIndex0 = false % Use index input of encoder

        ResetErrors0 = true; % Reset all error codes

        EnableError0Output = false; % Enable error output

    end

    properties(Nontunable)
        ControlMode0 = 'Position' % Control Mode
        
        MaxInputParameters = 20; % Maximum number of inputs
        MaxOutputParameters = 20; % Maximum number of outputs
    end
    
    properties(Constant, Hidden)
        ControlMode0Set = matlab.system.StringSet({'Position','Velocity','Current'})
%         ControlMode1Set = matlab.system.StringSet({'Position','Velocity','Current'})
        AutocalibrationSet = matlab.system.StringSet({'Disabled (fail if not calibrated)','Autocalibrate','Autocalibrate and store'})
    end

    properties(Nontunable)
        VelocityLimit0 = 20*pi; % Velocity limit [rad/s]
        CurrentLimit0 = 40; % Current limit [A]
        
        CountsPerRotate0 = 8192; % Counts per rotate of encoder
        CountsPerRotate1 = 8192; % Counts per rotate of encoder
       
        Inputs = {}; % Aditional inputs
        Outputs = {}; % Aditional outputs
    end
    
    properties(Nontunable, Logical)
        enableVBusRead = false
    end

    properties (Access = private)
        inputParameters = {}
        outputParameters = {}
        inputNames = {}
        outputNames = {}
        inputMultiplier = ones(1, 0);
        outputMultiplier = ones(1, 0);
        portFilePointer = 0;
        odrive_module = 0;
        odrive_usdef = 0;
    end

    

    methods
        % Constructor
        function obj = system_odrive_v3(varargin)
            % Support name-value pair arguments when constructing object
            setProperties(obj,nargin,varargin{:})
        end
    end

    methods (Access = protected)
        %% Common functions
        function setupImpl(obj)
            % Perform one-time calculations, such as computing constants
            %%%%%%Initial Setup%%%%%%
            [~, obj.inputParameters, obj.inputMultiplier, ~] = obj.generateInputs();
            [~, obj.outputParameters, obj.outputMultiplier, ~] = obj.generateOutputs();
            
			portname = "COM5";
            %Find motor
            try
                obj.portFilePointer = obj.odrive_connect(portname, 115200);
            end
            parameter = "vbus_voltage";
            a = obj.odrive_read_float(obj.portFilePointer, parameter);
		    disp(a)
			%Check if the port is functional
            %catch e
            %obj.portFilePointer = e.message;
            %end
            
            %err_type = 'Python Error: TimeoutError';
            %if (strcmp(obj.portFilePointer, err_type))
            %    error('Error. \n %s \n Odrive Not connected',err_type)                 
            %end
            
            %Other
            motor0_calibrated = false;
            encoder0_ready = false;

            if obj.EnableAxis0
                %Reset the errors to 0
                if obj.ResetErrors0
                    obj.odrive_write_int(obj.portFilePointer, "axis0.motor.error", 0);
					obj.odrive_write_int(obj.portFilePointer, "axis0.encoder.error", 0);
					obj.odrive_write_int(obj.portFilePointer, "axis0.controller.error", 0);
					obj.odrive_write_int(obj.portFilePointer, "axis0.error", 0);
                end
                %Other configs 
                % Encoder - Use Index - currentlim
				
                obj.odrive_write_int(obj.portFilePointer, "axis0.encoder.config.use_index", int32(obj.UseIndex0));
                obj.odrive_write_int(obj.portFilePointer, "axis0.motor.config.current_lim", obj.CurrentLimit0);
                vel_limit = (obj.VelocityLimit0*obj.CountsPerRotate0)/(2*pi);
                obj.odrive_write_int(obj.portFilePointer, "axis0.controller.config.vel_limit", vel_limit);
                %Check if motor/encoder is ready
                motor0_calibrated = obj.odrive_read_int(obj.portFilePointer, "axis0.motor.is_calibrated");
                encoder0_ready = obj.odrive_read_int(obj.portFilePointer, "axis0.encoder.is_ready");
            end

            
            %%%%Add if the the motor and encoder are not calibrated
            if (strcmp(obj.Autocalibration, 'Disabled (fail if not calibrated)'))
                if obj.EnableAxis0 && (~motor0_calibrated || ~encoder0_ready)
                    error('Error. \n Axis 0 not ready, need calibration')
                end
            end

            if startsWith(obj.Autocalibration, 'Autocalibrate')
                if obj.EnableAxis0
                    fprintf('heyy')
                    obj.odrive_write_int(obj.portFilePointer, "axis0.requested_state", int32(3));
                    pause(15)
                    %obj.autocalibration(obj.portFilePointer)
                    disp('Calibrating...')
					%CHeck ISSO
                    while int32(obj.odrive_read_int(obj.portFilePointer, 'axis0.motor.is_armed')) ~= 0
                        continue
                    end
                end
            
            end


            if obj.EnableAxis0
                switch(obj.ControlMode0)
                    case 'Position'
						obj.odrive_write_int(obj.portFilePointer, "axis0.controller.config.control_mode", int32(3));
                    case 'Velocity'
                        obj.odrive_write_int(obj.portFilePointer, "axis0.controller.config.control_mode", int32(2));
                    case 'Current'
                        obj.odrive_write_int(obj.portFilePointer, "axis0.controller.config.control_mode", int32(1));
                end
                %Closed Loop
                obj.odrive_write_int(obj.portFilePointer, "axis0.requested_state", int32(8));
            end
            
        end
        
        

        function varargout = stepImpl(obj, varargin)
            % Implement algorithm. Calculate y as a function of input u and
			%%%%%%%%%%MODIFICARR
            err_type = 'Python Error: TimeoutError';
            if (strcmp(obj.portFilePointer,  err_type))
                for ind = 1:nargout
                    varargout{ind} = 200+ind;
                end
            else
    
                if obj.EnableTiming
                    tic;
                end
                for ind = 1:(nargin-1)
                    value = varargin{ind}*obj.inputMultiplier(ind);
                    obj.odrive_quick_write(obj.portFilePointer,obj.inputParameters{ind}(1),0, value);
                    flush(obj.portFilePointer)
                end
                for ind = 1:nargout
%                     value = 0;
                    flush(obj.portFilePointer)
                    value = obj.odrive_read_float(obj.portFilePointer, obj.outputParameters{ind});
%                     fprintf(obj.outputMultiplier)
%                     fprintf(obj.outputMultiplier(ind))
%                     fprintf(ind)
                    value = value*obj.outputMultiplier(ind);
                    varargout{ind} = value;
                end
                if obj.EnableTiming
                    endTime = toc;
                    disp(endTime);
                end
            end
        end


        function releaseImpl(obj)
            % Release resources, such as file handles
            if obj.EnableAxis0
                switch(obj.ControlMode0)
                    case 'Velocity'
                        obj.odrive_write_int(obj.portFilePointer, "axis0.controller.input_vel", int32(0));
                    case 'Current'
					obj.odrive_write_int(obj.portFilePointer, "axis0.controller.input_torque", int32(0));
                end
            end
        end


        %Generate INPUTS - ControlMode mainly
        function [names, parameters, multipliers, count] = generateInputs(obj)
            count = 1;
            parameters = cell(1, obj.MaxInputParameters);
            names = cell(1, obj.MaxInputParameters);
            multipliers = ones(1, obj.MaxInputParameters);
            if obj.EnableAxis0
                names{count} = ['Axis 0 Reference ', lower(obj.ControlMode0)];
                %Transforms into p or v
                parameters{count} = lower(obj.ControlMode0(1));
%                 if parameters{count} == 'v' || parameters{count} == 'p'
%                     multipliers(count) = obj.CountsPerRotate0/(2*pi);
%                 end
                count=count+1;
            end
            
            if ~isempty(obj.Inputs)
                for ind = 1:length(obj.Inputs)
                    parameters{count} = obj.Inputs{ind};
                    names{count} = obj.Inputs{ind};
                    count=count+1;
                end
            end
            
            if (count-1) > obj.MaxInputParameters
                error("Too many inputs, disable some or increase maximum inputs constant in ODrive module");
            end
            
            assert(length(names) == length(parameters));
            
            for ind = count:obj.MaxInputParameters
                names{ind} = '';
                parameters{ind} = '';
            end
            count = count - 1;
        end

        %Output
        function [names, parameters, multipliers, count] = generateOutputs(obj)
            count = 1;
            parameters = cell(1,obj.MaxOutputParameters);
            names = cell(1,obj.MaxOutputParameters);
            multipliers = ones(1, obj.MaxOutputParameters);
            if obj.EnableAxis0
                if obj.EnablePosition0Output
                    parameters{count} = 'axis0.encoder.pos_estimate';
                    names{count} = 'Axis 0 Estimated position [rad]';
%                     multipliers(count) = (2*pi)/obj.CountsPerRotate0;
                    count=count+1;
                end

                if obj.EnableVelocity0Output
                    parameters{count} = 'axis0.encoder.vel_estimate';
                    names{count} = 'Axis 0 Estimated velocity [rad/s]';
%                     multipliers(count) = (2*pi)/obj.CountsPerRotate0;
                    count=count+1;
                end
                
                if obj.EnableCurrent0Output
                    parameters{count} = 'axis0.motor.current_control.Iq_measured';
                    names{count} = 'Axis 0 Measured current [A]';
                    count=count+1;
                end

                if obj.EnableError0Output
                    parameters{count} = 'axis0.error';
                    names{count} = 'Axis 0 Error state';
                    count=count+1;
                end
            end

            if obj.EnableAxis1
                if obj.EnablePosition1Output
                    parameters{count} = 'axis1.encoder.pos_estimate';
                    names{count} = 'Axis 1 Estimated position [rad]';
%                     multipliers(count) = (2*pi)/obj.CountsPerRotate1;
                    count=count+1;
                end
            end


            if obj.EnableVbusOutput
                parameters{count} = 'vbus_voltage';
                names{count} = 'Bus voltage [V]';
                count=count+1;
            end
            
             if ~isempty(obj.Outputs)
                for out = obj.Outputs
                    parameters{count} = out{1};
                    names{count} = out{1};
                    count=count+1;
                end
            end
            
            if (count-1) > obj.MaxOutputParameters
                error("Too many outputs, disable some or increase maximum outputs constant in ODrive module");
            end
            
            for ind = count:obj.MaxOutputParameters
                names{ind} = '';
                parameters{ind} = '';
            end
            count = count - 1;
        end


        

%         %% Backup/restore functions
%         function s = saveObjectImpl(obj)
%             % Set properties in structure s to values in object obj
% 
%             % Set public properties and states
%             s = saveObjectImpl@matlab.System(obj);
% 
%             % Set private and protected properties
%             %s.myproperty = obj.myproperty;
%         end
% 
%         function loadObjectImpl(obj,s,wasLocked)
%             % Set properties in object obj to values in structure s
% 
%             % Set private and protected properties
%             % obj.myproperty = s.myproperty; 
% 
%             % Set public properties and states
%             loadObjectImpl@matlab.System(obj,s,wasLocked);
%         end

        %% Simulink functions

        function num = getNumInputsImpl(obj)
            % Define total number of inputs for system with optional inputs
            [~,~,~,num] = obj.generateInputs();
        end

        function num = getNumOutputsImpl(obj)
            [~,~,~,num] = obj.generateOutputs();
        end

        function flag = isInputSizeMutableImpl(~,~)
            flag = false;
        end

        function varargout = isInputFixedSizeImpl(~,~)
            varargout{1} = true;
        end

        function flag = isInputComplexityMutableImpl(~,~)
            flag = false;
        end

        function validateInputsImpl(obj, varargin)
        end

        function validatePropertiesImpl(obj)
        end

        function icon = getIconImpl(obj)
            % Define icon for System block
            icon = mfilename("class"); % Use class name
            % icon = "My System"; % Example: text icon
            % icon = ["My","System"]; % Example: multi-line text icon
            % icon = matlab.system.display.Icon("myicon.jpg"); % Example: image file icon
        end

        function varargout = getInputNamesImpl(obj)
            % Return input port names for System block
            [varargout,~,~,~] = obj.generateInputs();
        end

        function varargout = getOutputNamesImpl(obj)
            % Return output port names for System block
            [varargout,~,~,~] = obj.generateOutputs();
        end

        function varargout = getOutputSizeImpl(obj)
            varargout = cell(1,nargout);
            for k = 1:nargout
                varargout{k} = [1 1];
            end
        end

        function varargout = getOutputDataTypeImpl(obj)
            % Return data type for each output port
            varargout = cell(1,nargout);
            for k = 1:nargout
                varargout{k} = "double";

                % Example: inherit data type from first input port
                % varargout{k} = propagatedInputDataType(obj,1);
            end
        end

        function varargout = isOutputComplexImpl(obj)
            % Return true for each output port with complex data
            varargout = cell(1,nargout);
            for k = 1:nargout
                varargout{k} = false;

                % Example: inherit complexity from first input port
                % varargout{k} = propagatedInputComplexity(obj,1);
            end
        end

        function varargout = isOutputFixedSizeImpl(obj)
            % Return true for each output port with fixed size
            varargout = cell(1,nargout);
            for k = 1:nargout
                varargout{k} = true;

                % Example: inherit fixed-size status from first input port
                % varargout{k} = propagatedInputFixedSize(obj,1);
            end
        end
        
        end
    

    methods (Static)
        function name = getDescriptiveName()
            name = 'Odrive';
        end
    end

    methods (Static, Access = protected)
	
        %% Simulink customization functions
        function header = getHeaderImpl
            % Define header panel for System block dialog
            header = matlab.system.display.Header(mfilename("class"), ...
                'Title', Odrive.getDescriptiveName(), ...
                'Text', 'This Matlab System controls and reads Odrive \n Position control the input must be in rad ');
        end
        
        function groups = getPropertyGroupsImpl
            % Define property section(s) for System block dialog
            % group = matlab.system.display.Section(mfilename("class"));
           configGroup = matlab.system.display.Section(...
               'Title','ODrive configuration',...
               'PropertyList',{'Autocalibration', 'EnableTiming'});
                     
           axis0Group = matlab.system.display.SectionGroup(...
               'Title','Axis 0', ...
               'PropertyList',{'EnableAxis0','UseIndex0','ResetErrors0','ControlMode0','VelocityLimit0', 'CurrentLimit0', 'CountsPerRotate0', 'EnablePosition0Output', 'EnableVelocity0Output', 'EnableCurrent0Output', 'EnableError0Output'});
           
           axis1Group = matlab.system.display.SectionGroup(...
               'Title','Others', ...
               'PropertyList',{'EnableAxis1', 'CountsPerRotate1', 'EnablePosition1Output'});

           inputsGroup = matlab.system.display.SectionGroup(...
               'Title','Inputs', ...
               'PropertyList',{'MaxInputParameters','Inputs'});
           
           outputsGroup = matlab.system.display.SectionGroup(...
               'Title','Outputs', ...
               'PropertyList',{'MaxOutputParameters','EnableVbusOutput','Outputs'});

           groups = [configGroup, axis0Group, axis1Group, inputsGroup, outputsGroup];
        end
        
        function simMode = getSimulateUsingImpl(~)
            simMode = 'Interpreted execution';
        end

        function flag = showSimulateUsingImpl
            % Return false if simulation mode hidden in System block dialog
            flag = false;
        end

    
	
	
		%Connect to Odrive
		function serial_odrive = odrive_connect(portname, baudrate)
			serialportObj = serialport(portname,baudrate);
			serialportObj.BaudRate = 115200;
			serialportObj.Timeout = 1;
			serial_odrive = serialportObj;  
		end
		
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
		
		function output = odrive_write_float(fd, parameter, value)
			flush(fd);
			writeline(fd, sprintf('w %s %f\n', parameter, value));
			output = [];
        end

        function output = odrive_quick_write(fd, type, axis, value)
			flush(fd);
			writeline(fd, sprintf('%c %d %f\n', type, axis , value));
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
		
		function vel = set_velocity(fd, motor_number, velocity, current_feedforward)
			flush(fd);
			writeline(fd, sprintf('v %i %f %f\n',motor_number, velocity, current_feedforward));
			vel = readline(f);
		end
		%Position

		function pos = set_position(fd, motor_number, velocity, current_feedforward)
			flush(fd);
			writeline(fd, sprintf('v %i %f %f\n',motor_number, velocity, current_feedforward));
			pos = readline(f);
		end
		
		function state = run_state(fd, axis, requested_state, wait_for_idle)
			flush(fd);
			writeline(fd, sprintf("w axis%d.requested_state %d\n", axis, requested_state));
			disp("Message sent");
			state = 0;
			if wait_for_idle
				pause(100);
				req_state_parameter = sprintf("axis%d.current_state", axis);
				state = odrive_read_int(fd, req_state_parameter);
			end
		end
		
		function req_state =  odrive_request_state(fp,  axis, requested_state)
			parameter = sprintf("axis%d.requested_state", axis);
			odrive_write_int(fp, parameter, requested_state);
			req_state = 0;
		end
		
		function wait_state = odrive_wait_for_state(fp, axis, requested_state, usleep, n_timeout)
			parameter = sprintf("axis%d.current_state", axis);
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
		
		function autocalib = autocalibration(f)
            fprintf("Calibrating...")
            %Confirmar
            fprintf('trying')
            obj.odrive_write_int(f, "axis0.requested_state", int32(3));
            fprintf('\n trying')
            pause(10)
        %     while motor.axis0.motor.is_armed:
        %         pass
            autocalib = odrive_read_int(f, "axis0.error");
            fprintf(autocalib)
        end
    end
end
