classdef kinematicTrajectoryNEW < matlab.System & ...
        fusion.scenario.internal.mixin.PlatformTrajectory ...
        & scenario.mixin.Perturbable
%KINEMATICTRAJECTORY Rate-driven trajectory generator
%   TRAJ = KINEMATICTRAJECTORY returns a System object, TRAJ, that 
%   generates a trajectory based on acceleration and angular velocity.
%
%   TRAJ = KINEMATICTRAJECTORY('Name', Value, ...) returns a 
%   KINEMATICTRAJECTORY System object with each specified property name set
%   to the specified value. You can specify additional name-value pair 
%   arguments in any order as (Name1,Value1,...,NameN, ValueN).
%   
%   Step method syntax:
%
%   [POS, ORIENT, VEL, ACC, ANGVEL] = step(TRAJ, ACCBODY, ANGVELBODY) 
%   outputs the trajectory state based on acceleration (ACCBODY) and 
%   angular velocity (ANGVELBODY).
%
%   The inputs to KINEMATICTRAJECTORY are defined as follows:
%
%       ACCBODY       Driving acceleration in the body coordinate system 
%                     specified as a real finite N-by-3 array in meters per
%                     second squared. N is the number of samples in the 
%                     current frame. 
%
%       ANGVELBODY    Driving angular velocity in the body coordinate 
%                     system specified as a real finite N-by-3 array in 
%                     radians per second. N is the number of samples in the
%                     current frame. 
%
%   The outputs of KINEMATICTRAJECTORY are defined as follows:
%
%       POS           Position in the local navigation coordinate system 
%                     specified as a real finite N-by-3 array in meters. N
%                     is the number of samples in the current frame.
%
%       ORIENT        Orientation with respect to the local navigation 
%                     coordinate system specified as a quaternion N-element
%                     column vector or a 3-by-3-by-N rotation matrix. Each
%                     quaternion or rotation matrix is a frame rotation
%                     from the local navigation coordinate system to the
%                     current body coordinate system. N is the number of
%                     samples in the current frame.
%
%       VEL           Velocity in the local navigation coordinate system 
%                     specified as a real finite N-by-3 array in meters per
%                     second. N is the number of samples in the current
%                     frame.
%
%       ACC           Acceleration in the local navigation coordinate 
%                     system specified as a real finite N-by-3 array in
%                     meters per second squared. N is the number of samples
%                     in the current frame.
%
%       ANGVEL        Angular velocity in the local navigation coordinate 
%                     system specified as a real finite N-by-3 array in
%                     radians per second. N is the number of samples in the
%                     current frame.
%
%   Either single or double datatypes are supported for the inputs to 
%   KINEMATICTRAJECTORY. Outputs have the same datatype as the input.
%
%   System objects may be called directly like a function instead of using
%   the step method. For example, y = step(obj, x) and y = obj(x) are
%   equivalent.
%
%   KINEMATICTRAJECTORY methods:
%
%   step           - See above description for use of this method
%   perturbations  - Define perturbations to the trajectory
%   perturb        - Apply perturbations to the trajectory
%   clone          - Create KINEMATICTRAJECTORY object with same property 
%                    values
%
%   KINEMATICTRAJECTORY properties:
%
%   SampleRate               - Sample rate of trajectory (Hz)
%   Position                 - Position state (m)
%   Orientation              - Orientation state
%   Velocity                 - Velocity state (m/s)
%   Acceleration             - Acceleration state (m/s^2)
%   AngularVelocity          - Angular velocity state (rad/s)
%   SamplesPerFrame          - Number of samples per output frame
%   AccelerationSource       - Source of acceleration state
%   AngularVelocitySource    - Source of angular velocity state
%
%   % EXAMPLE 1: Generate a circular trajectory with inputs.
%
%   N = 10000;
%   Fs = 100;
%   r = 10;
%   speed = 2.5;
%   initialYaw = 90;
% 
%   initPos = [r, 0, 0];
%   initVel = [0, speed, 0];
%   initAtt = quaternion([initialYaw, 0, 0], 'eulerd', 'ZYX', 'frame');
% 
%   traj = kinematicTrajectory('SampleRate', Fs, ...
%       'Position', initPos, ...
%       'Velocity', initVel, ...
%       'Orientation', initAtt);
% 
%   accBody = [0 speed^2/r 0];
%   angvelBody = [0 0 speed/r];
% 
%   pos = zeros(N, 3);
%   q = quaternion.zeros(N, 1);
% 
%   for i = 1:N
%       [pos(i,:), q(i)] = traj(accBody, angvelBody);
%   end
% 
%   plot3(pos(:,1), pos(:,2), pos(:,3))
%   title('Position')
%   xlabel('X (m)')
%   ylabel('Y (m)')
%   zlabel('Z (m)')
%
%   % EXAMPLE 2: Generate a spiraling circular trajectory with no inputs.
%
%   N = 10000;
%   Fs = 100;
%   r = 10;
%   speed = 2.5;
%   initialYaw = 90;
% 
%   initPos = [r 0 0];
%   initVel = [0 speed 0];
%   initOrient = quaternion([initialYaw 0 0], 'eulerd', 'ZYX', 'frame');
%
%   accBody = [0 speed^2/r 0.01];
%   angVelBody = [0 0 speed/r];
% 
%   traj = kinematicTrajectory('SampleRate', Fs, ...
%       'Position', initPos, ...
%       'Velocity', initVel, ...
%       'Orientation', initOrient, ...
%       'AccelerationSource', 'Property', ...
%       'Acceleration', accBody, ...
%       'AngularVelocitySource', 'Property', ...
%       'AngularVelocity', angVelBody);
% 
%   pos = zeros(N, 3);
%   for i = 1:N
%       pos(i,:) = traj();
%   end
% 
%   plot3(pos(:,1), pos(:,2), pos(:,3))
%   title('Position')
%   xlabel('X (m)')
%   ylabel('Y (m)')
%   zlabel('Z (m)')
%
%   See also WAYPOINTTRAJECTORY

%   Copyright 2018-2020 The MathWorks, Inc.

%#codegen

    properties
        % SampleRate Sampling rate (Hz)
        % Specify the sampling frequency of the trajectory as a positive 
        % scalar. This property is tunable. The default value is 100.
        SampleRate = 100;
        % Position Position state (m)
        % Specify the position in the local frame as a real 3-element row 
        % vector. This property is tunable. The default initial value is 
        % [0 0 0].
        Position = [0 0 0];
    end
    
    properties (Dependent)
        % Orientation Orientation state 
        % Specify the orientation as a scalar quaternion or a double or 
        % single 3-by-3 rotation matrix. The orientation is a frame 
        % rotation from the local navigation coordinate system to the 
        % current body frame. This property is tunable. The default initial
        % value is quaternion(1,0,0,0).
        Orientation;
    end
    
    properties (Hidden, SetAccess = protected)
        CurrentPosition
        CurrentVelocity
        CurrentOrientation
        CurrentAcceleration
        CurrentAngularVelocity
	end

    properties
        % Velocity Velocity state (m/s)
        % Specify the velocity in the navigation frame as a real 3-element 
        % row vector. This property is tunable. The default initial value
        % is [0 0 0].
        Velocity = [0 0 0];
        % Acceleration Acceleration state (m/s^2)
        % Specify the acceleration in the body frame as a real 3-element
        % row vector. This property is tunable. The default initial value
        % is [0 0 0].
        Acceleration = [0 0 0];
        % AngularVelocity Angular velocity state (rad/s)
        % Specify the angular velocity in the body frame as a real
        % 3-element row vector. This property is tunable. The default
        % initial value is [0 0 0].
        AngularVelocity = [0 0 0];
    end

    properties (Nontunable, PositiveInteger)
        % SamplesPerFrame Number of samples per output frame
        % Specify the number of samples to buffer into each trajectory
        % output frame. The default value is 1.
        SamplesPerFrame = 1;
    end

    properties (Hidden, SetAccess = protected)
        CurrentPoseValid = true
	end

    properties (Nontunable)
        % AccelerationSource Source of acceleration state
        % Specify the source of the acceleration as one of 'Input' |
        % 'Property'. The default value is 'Input'.
        AccelerationSource = 'Input';
        % AngularVelocitySource Source of angular velocity state
        % Specify the source of the angular velocity as one of 'Input' |
        % 'Property'. The default value is 'Input'.
        AngularVelocitySource = 'Input';
    end

    properties (Access = private)
        pOrientation;
    end

    properties (Access = private, Nontunable)
        IsOrientationQuaternion = true;
    end
    
    properties (Constant, Hidden)
        AccelerationSourceSet = matlab.system.StringSet( ...
            {'Input', 'Property'});
        AngularVelocitySourceSet = matlab.system.StringSet( ...
            {'Input', 'Property'});
    end

    properties (Access = private)
        pInitialPosition;
        pInitialVelocity;
        pInitialOrientation;
        pInitialAcceleration;
        pInitialAngularVelocity;
    end
    
    methods
        function obj = kinematicTrajectory(varargin)
            setProperties(obj, nargin, varargin{:});
            isOrientationSpecified = false;
            for i = 1:2:nargin
                if strcmp(varargin{i}, 'Orientation')
                    isOrientationSpecified = true;
                    obj.CurrentOrientation = obj.Orientation;
                    break;
                end
            end
            if ~isOrientationSpecified
                obj.pOrientation = quaternion(1, 0, 0, 0);
            end
            obj.CurrentAcceleration = [0 0 0];
            obj.CurrentAngularVelocity = [0 0 0];
        end

        function val = get.Orientation(obj)
            q = obj.pOrientation;
            if obj.IsOrientationQuaternion
                val = q;
            else
                val = rotmat(q,%% 'frame');
            end
        end

        function set.Orientation(obj, val)
            isQuat = isa(val, 'quaternion');
            obj.IsOrientationQuaternion = isQuat;
            expectedDataTypes = {'double', 'single'};
            if isQuat
                % Validate quaternion.
                validateattributes(val, {'quaternion'}, ...
                    {'finite', 'scalar'}, '', 'Orientation'); 
                obj.pOrientation = val;
            else
                % Validate rotation matrix.
                validateattributes(val, expectedDataTypes, ...
                    {'real', 'finite', '2d', 'size', [3 3]}, ...
                    '', 'Orientation');
                obj.pOrientation = quaternion(val, 'rotmat', 'frame');
            end
        end
        
        function set.SampleRate(obj, val)
            validateattributes(val, {'double', 'single'}, ...
                {'real', 'scalar', 'positive', 'finite'}, ...
                '', ...
                'SampleRate');
            obj.SampleRate = val;
        end
        
        function set.Position(obj, val)
            validateattributes(val, {'double', 'single'}, ...
                {'real', 'finite', 'numel', 3}, ...
                '', ...
                'Position');
            obj.Position = val(:).';
        end
        
        function set.Velocity(obj, val)
            validateattributes(val, {'double', 'single'}, ...
                {'real', 'finite', 'numel', 3}, ...
                '', ...
                'Velocity');
            obj.Velocity = val(:).';
        end
        
        function val = get.CurrentPosition(obj)
            val = obj.Position;
        end
        
        function val = get.CurrentVelocity(obj)
            val = obj.Velocity;
        end
        function val = get.CurrentAcceleration(obj)
            val = obj.Acceleration;
        end
        function val = get.CurrentOrientation(obj)
            val = obj.Orientation;
        end
        function val = get.CurrentAngularVelocity(obj)
            val = obj.AngularVelocity;
        end
        function set.Acceleration(obj, val)
            validateattributes(val, {'double', 'single'}, ...
                {'real', 'finite', 'numel', 3}, ...
                '', ...
                'Acceleration');
            obj.Acceleration = val(:).';
        end
        function set.AngularVelocity(obj, val)
            validateattributes(val, {'double', 'single'}, ...
                {'real', 'finite', 'numel', 3}, ...
                '', ...
                'AngularVelocity');
            obj.AngularVelocity = val(:).';
        end
    end
    
    methods (Access = protected)
        function setupImpl(obj, varargin)
            obj.pInitialPosition = obj.Position;
            obj.pInitialVelocity = obj.Velocity;
            obj.pInitialOrientation = obj.Orientation;
            obj.pInitialAcceleration = obj.Acceleration;
            obj.pInitialAngularVelocity = obj.AngularVelocity;
        end

        function [pos, orient, vel, acc, av] = stepImpl(obj, varargin)

            numargs = getNumInputs(obj);
            if (numargs == 2)
                accB = varargin{1};
                avB = varargin{2};
                spf = size(accB, 1);
            elseif (numargs == 0)
                spf = obj.SamplesPerFrame;
                accBSample = obj.Acceleration;
                accB = bsxfun(@times, accBSample, ones(spf, 3, 'like', accBSample));
                avBSample = obj.AngularVelocity;
                avB = bsxfun(@times, avBSample, ones(spf, 3, 'like', avBSample));
            elseif (numargs == 1) && strcmp(obj.AccelerationSource, 'Input')
                accB = varargin{1};
                spf = size(accB, 1);
                avBSample = obj.AngularVelocity;
                avB = bsxfun(@times, avBSample, ones(spf, 3, 'like', accB));
            else % (numargs == 1) && strcmp(obj.AngularVelocitySource, 'Input')
                avB = varargin{1};
                spf = size(avB, 1);
                accBSample = obj.Acceleration;
                accB = bsxfun(@times, accBSample, ones(spf, 3, 'like', avB));
            end

            isOrientationQuaternion = obj.IsOrientationQuaternion;
            if (spf > 0)
                dt = 1 ./ obj.SampleRate;
                q = quaternion(avB .* dt, 'rotvec');
                prevQ = obj.pOrientation;
                q(1) = prevQ .* q(1);

                for i = 2:numel(q)
                    q(i) = q(i-1) .* q(i);
                end

                obj.pOrientation = q(spf);
                if isOrientationQuaternion
                    orient = [prevQ; q(1:spf-1,:)];
                else
                    orient = rotmat([prevQ; q(1:spf-1,:)], 'frame');
                end
                q = [prevQ; q(1:spf-1,:)];
                
                av = rotatepoint(q, avB);
                acc = rotatepoint(q, accB);

                prevVel = obj.Velocity;
                vel = acc .* dt;
                vel(1,:) = prevVel + vel(1,:);
                vel = cumsum(vel, 1);
                obj.Velocity = vel(spf,:);
                vel = [prevVel; vel(1:spf-1,:)];

                prevPos = obj.Position;
                pos = vel .* dt + 0.5 .* acc .* dt.^2;
                pos(1,:) = prevPos + pos(1,:);
                pos = cumsum(pos, 1);
                obj.Position = pos(spf,:);
                pos = [prevPos; pos(1:spf-1,:)];
                obj.CurrentAcceleration = double(acc(end,:));
                obj.CurrentAngularVelocity = double(av(end,:));
            else % (spf == 0)
                emptyOutput = zeros(0, 3, 'like', accB);
                pos = emptyOutput;
                if isOrientationQuaternion
                    orient = quaternion.zeros(0, 1, 'like', emptyOutput);
                else
                    orient = zeros(0, 3, 3, 'like', emptyOutput);
                end
                vel = emptyOutput;
                acc = emptyOutput;
                av = emptyOutput;
            end
        end
        
        function validateInputsImpl(obj, varargin)
            numargs = getNumInputs(obj);
            if (numargs == 2)
                accB = varargin{1};
                avB = varargin{2};
                validateattributes(accB, {'double','single'}, ...
                    {'real','finite', '2d', 'ncols', 3}, ...
                    '', ...
                    'accBody');
                validateattributes(avB, {'double','single'}, ...
                    {'real','finite', '2d', 'nrows', size(accB, 1), 'ncols', 3}, ...
                    '', ...
                    'angvelBody');
            elseif (numargs == 1) && strcmp(obj.AccelerationSource, 'Input')
                validateattributes(varargin{1}, {'double','single'}, ...
                    {'real','finite', '2d', 'ncols', 3}, ...
                    '', ...
                    'accBody');
            elseif (numargs == 1) && strcmp(obj.AngularVelocitySource, 'Input')
                validateattributes(varargin{1}, {'double','single'}, ...
                    {'real','finite', '2d', 'ncols', 3}, ...
                    '', ...
                    'angvelBody');
            end
        end
        
        function flag = isInputComplexityMutableImpl(~, ~)
            flag = false;
        end

        function num = getNumInputsImpl(obj)
            num = 0;
            if strcmp(obj.AccelerationSource, 'Input')
                num = num + 1;
            end
            if strcmp(obj.AngularVelocitySource, 'Input')
                num = num + 1;
            end
        end
        
        function flag = isInactivePropertyImpl(obj, prop)
            flag = false;
            if strcmp(prop, 'Acceleration')
                if strcmp(obj.AccelerationSource, 'Input')
                    flag = true;
                end
            end
            if strcmp(prop, 'AngularVelocity')
                if strcmp(obj.AngularVelocitySource, 'Input')
                    flag = true;
                end
            end
            if strcmp(prop, 'SamplesPerFrame')
                if strcmp(obj.AccelerationSource, 'Input') ...
                    || strcmp(obj.AngularVelocitySource, 'Input')
                    flag = true;
                end
            end
        end
        
        function s = saveObjectImpl(obj)
            % Save public properties.
            s = saveObjectImpl@matlab.System(obj);
            
            % Save perturbation related properties
            s = savePerts(obj, s);

            % Save private properties. 
            s.pOrientation = obj.pOrientation;
            s.IsOrientationQuaternion = obj.IsOrientationQuaternion;
            if isLocked(obj)
                s.pInitialPosition = obj.pInitialPosition;
                s.pInitialVelocity = obj.pInitialVelocity;
                s.pInitialOrientation = obj.pInitialOrientation;
                s.pInitialAcceleration = obj.pInitialAcceleration;
                s.pInitialAngularVelocity = obj.pInitialAngularVelocity;
            end
        end
        
        function loadObjectImpl(obj, s, wasLocked)
            % Load public properties. 
            loadObjectImpl@matlab.System(obj, s, wasLocked);
            
            % Load perturbation related properties
            loadPerts(obj, s);

            % Load private properties.
            obj.pOrientation = s.pOrientation;
            obj.IsOrientationQuaternion = s.IsOrientationQuaternion;
            if wasLocked
                obj.pInitialPosition = s.pInitialPosition;
                obj.pInitialVelocity = s.pInitialVelocity;
                obj.pInitialOrientation = s.pInitialOrientation;
                obj.pInitialAcceleration = s.pInitialAcceleration;
                obj.pInitialAngularVelocity = s.pInitialAngularVelocity;
            end
        end
        
        function perts = defaultPerturbations(~)
            perturbableProps = {"Position", "Velocity"}; %#ok<CLARRSTR>
            perts = struct(...
                'Property', perturbableProps, ...
                'Type', "None", ...
                'Value', {{NaN, NaN}}...
                );
        end
    end
    
    methods (Hidden)
        function restart(obj)
            reset(obj);
            if isLocked(obj)
                obj.Position = obj.pInitialPosition;
                obj.Velocity = obj.pInitialVelocity;
                obj.Orientation = obj.pInitialOrientation;
                obj.Acceleration = obj.pInitialAcceleration;
                obj.AngularVelocity = obj.pInitialAngularVelocity;
            end
        end
        
        function initTrajectory(obj)
            release(obj);
            setup(obj);
        end
        
        function initUpdateRate(obj, newUpdateRate)
            if obj.SampleRate ~= newUpdateRate
                obj.SampleRate = newUpdateRate;
            end
        end
        
        function status = move(obj, ~)
            n = getNumInputs(obj);
            if (n == 2)
                step(obj, obj.CurrentAcceleration, obj.CurrentAngularVelocity);
            elseif (n == 0)
                step(obj);
            else
                if strcmp(obj.AccelerationSource, 'Input')
                    step(obj, obj.CurrentAcceleration);
                else
                    step(obj, obj.CurrentAngularVelocity);
                end
            end
            status = true;
        end
    end
    
    methods(Static, Hidden)
        function flag = isAllowedInSystemBlock
            flag = false;
        end
    end
end