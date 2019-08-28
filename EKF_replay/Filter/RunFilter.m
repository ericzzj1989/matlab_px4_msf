function output = RunFilter(param,vicon_imu_data,mag_data,baro_data,gps_data,vicon_pose_data,varargin)

% compulsory inputs

% param : parameters defined  by SetParameterDefaults.m
% vicon_imu_data : IMU delta angle and velocity data in body frame
% mag_data : corrected magnetometer field measurements in body frame
% baro_data : barometric height measurements
% gps_data : GPS NED pos vel measurements in local earth frame
% pose_data : Camara pose from ORB-SLAM or SVO

% optional inputs

% rng _data : measurements for a Z body axis aligned range finder
% flow_data : XY axis optical flow angular rate measurements in body frame
% viso_data : ZED camera visula odometry measurements

if 0
nVarargs = length(varargin);
if nVarargs >= 2
    flowDataPresent = ~isempty(varargin{1}) && ~isempty(varargin{2});
    rng_data = varargin{1};
    flow_data = varargin{2};
    if flowDataPresent
        fprintf('Using optical Flow Data\n',nVarargs);
    end
else
    flowDataPresent = 0;
end

if nVarargs >= 3
    visoDataPresent = ~isempty(varargin{3});
    viso_data = varargin{3};
    if visoDataPresent
        fprintf('Using ZED camera odometry data\n',nVarargs);
    end
else
    visoDataPresent = 0;
end

end


%% Set initial conditions

% constants
deg2rad = pi/180;
gravity = 9.80665; % initial value of gravity - will be updated when WGS-84 position is known

% initialise the state vector
[states, imu_start_index, pose_start_index] = InitStates(param,vicon_imu_data,gps_data,mag_data,baro_data,vicon_pose_data);

dt_imu_avg = median(vicon_imu_data.dt_imu)*1e-9;
indexStop = length(vicon_imu_data.dt_imu) - imu_start_index;
indexStart = 1;

% create static structs for output data
output = struct('time_lapsed',[]',...
    'euler_angles',[],...
    'velocity_NED',[],...
    'position_NED',[],...
    'gyro_bias',[],...
    'accel_bias',[],...
    'mag_NED',[],...
    'mag_XYZ',[],...
    'wind_NE',[],...
    'lamda',[],...
    'pos_ci',[],...
    'quat_ic',[],...
    'pos_vw',[],...
    'quat_vw',[],...
    'pos_cv',[],...
    'quat_vc',[],...
    'dt',0,...
    'state_variance',[],...
    'innovations',[],...
    'magFuseMethod',[]);

% initialise the state covariance matrix
covariance = InitCovariance(param,dt_imu_avg,1,gps_data);

if (param.fusion.camIsAbsolute == 0)
    states_old = states;
    covariance_old = covariance;
    F_old = calcF39(0,0,0,0,0,0,dt_imu_avg,0,0,0,0,0,0,states(1),states(2),states(3),states(4));
    measurements_old = [vicon_pose_data.q0(pose_start_index);vicon_pose_data.q1(pose_start_index);vicon_pose_data.q2(pose_start_index);vicon_pose_data.q3(pose_start_index);vicon_pose_data.x(pose_start_index);vicon_pose_data.y(pose_start_index);vicon_pose_data.z(pose_start_index)];
end


%% Main Loop

% control flags
% gps_use_started = boolean(false);
% gps_use_blocked = boolean(false);
% camera_use_blocked = boolean(false);
% viso_use_blocked = boolean(false);
% flow_use_blocked = boolean(false);

% array access index variables
imuIndex = imu_start_index;
poseIndex = pose_start_index;
% last_gps_index = 0;
% gps_fuse_index = 0;
% last_baro_index = 0;
% baro_fuse_index = 0;
% last_mag_index = 0;
% mag_fuse_index = 0;
last_cam_index = 0;
cam_fuse_index = 0;
% last_flow_index = 0;
% flow_fuse_index = 0;
% last_viso_index = 0;
% viso_fuse_index = 0;
% last_range_index = 0;

% covariance prediction variables
delAngCov = [0;0;0]; % delta angle vector used by the covariance prediction (rad)
delVelCov = [0;0;0]; % delta velocity vector used by the covariance prediction (m/sec)
dtCov = 0; % time step used by the covariance prediction (sec)
dtCovInt = 0; % accumulated time step of covariance predictions (sec)
covIndex = 0; % covariance prediction frame counter

output.magFuseMethod = param.fusion.magFuseMethod;
output.camIsAbsolute = param.fusion.camIsAbsolute;
% range = 0.1;

% variables used to control dead-reckoning timeout
% last_drift_constrain_time = - param.control.velDriftTimeLim;
% last_synthetic_velocity_fusion_time = 0;
% last_valid_range_time = - param.fusion.rngTimeout;

for index = indexStart:indexStop
    
    % read IMU measurements
    local_time=(vicon_imu_data.time_us(imuIndex) - vicon_imu_data.time_us(indexStart)) * 1e-9;
    delta_angle = [vicon_imu_data.angular_vel_x(imuIndex);vicon_imu_data.angular_vel_y(imuIndex);vicon_imu_data.angular_vel_z(imuIndex)] * dt_imu_avg;
%     delta_angle(1,1) = vicon_imu_data.angular_vel_x(imuIndex) * dt_imu_avg;
%     delta_angle(2,1) = vicon_imu_data.angular_vel_y(imuIndex) * dt_imu_avg;
%     delta_angle(3,1) = vicon_imu_data.angular_vel_z(imuIndex) * dt_imu_avg;
    delta_velocity = [vicon_imu_data.accel_x(imuIndex);vicon_imu_data.accel_y(imuIndex);vicon_imu_data.accel_z(imuIndex)] * dt_imu_avg;
%     delta_velocity(1,1) = vicon_imu_data.accel_x(imuIndex) * dt_imu_avg;
%     delta_velocity(2,1) = vicon_imu_data.accel_y(imuIndex) * dt_imu_avg;
%     delta_velocity(3,1) = vicon_imu_data.accel_z(imuIndex) * dt_imu_avg;
    dt_imu = vicon_imu_data.dt_imu(imuIndex)*1e-9;
    imuIndex = imuIndex+1;

     position_cv = [vicon_pose_data.x(poseIndex);vicon_pose_data.y(poseIndex);vicon_pose_data.z(poseIndex)];
     quat_cv = [vicon_pose_data.q0(poseIndex);vicon_pose_data.q1(poseIndex);vicon_pose_data.q2(poseIndex);vicon_pose_data.q3(poseIndex)];
     dt_pose = vicon_pose_data.dt_pose(poseIndex) * 1e-9;
     poseIndex = poseIndex + 2;   

    % predict states
    [states, delAngCorrected, delVelCorrected]  = PredictStates(states,delta_angle,delta_velocity,vicon_imu_data.dt_imu(imuIndex)*1e-9,gravity,gps_data.refLLH(1,1)*deg2rad);
    
    % constrain states
    [states]  = ConstrainStates(states,dt_imu_avg);
    
    dtCov = dtCov + dt_imu;
    delAngCov = delAngCov + delAngCorrected;
    delVelCov = delVelCov + delVelCorrected;
%    if (dtCov > 0.02)
        % predict covariance
        [covariance, F] = PredictCovariance(delAngCov,delVelCov,states,covariance,dtCov,param);
        delAngCov = [0;0;0];
        delVelCov = [0;0;0];
        dtCovInt = dtCovInt + dtCov;
        dtCov = 0;
        covIndex = covIndex + 1;
        
        % output state data
        output.time_lapsed(covIndex) = local_time;
        output.euler_angles_wi(covIndex,:) = QuatToEul(states(1:4)')';
        output.velocity_NED(covIndex,:) = states(5:7)';
        output.position_NED_iw(covIndex,:) = states(8:10)';
        output.gyro_bias(covIndex,:) = states(11:13)';
        output.accel_bias(covIndex,:) = states(14:16)';
        output.mag_NED(covIndex,:) = states(17:19);
        output.mag_XYZ(covIndex,:) = states(20:22);
        output.wind_NE(covIndex,:) = states(23:24);
        output.lamda(covIndex,:) = states(25);
        output.position_ci(covIndex,:) = states(26:28)';
        output.euler_angles_ic(covIndex,:) = QuatToEul(states(29:32)')';
        output.position_vw(covIndex,:) = states(33:35)';
        output.euler_angles_vw(covIndex,:) = QuatToEul(states(36:39)')';
        output.position_cv(covIndex,:) = position_cv;
        output.euler_angles_vc(covIndex,:) = QuatToEul(quat_cv);
        
        % output covariance data
        for i=1:39
            output.state_variances(covIndex,i) = covariance(i,i);
        end
        
        % output equivalent euler angle variances
        error_transfer_matrix_wi = quat_to_euler_error_transfer_matrix(states(1),states(2),states(3),states(4));
        euler_covariance_matrix_wi = error_transfer_matrix_wi * covariance(1:4,1:4) * transpose(error_transfer_matrix_wi);
        for i=1:3
            output.euler_variances_wi(covIndex,i) = euler_covariance_matrix_wi(i,i);
        end

        error_transfer_matrix_ic = quat_to_euler_error_transfer_matrix(states(29),states(30),states(31),states(32));
        euler_covariance_matrix_ic = error_transfer_matrix_ic * covariance(29:32,29:32) * transpose(error_transfer_matrix_ic);
        for i=1:3
            output.euler_variances_ic(covIndex,i) = euler_covariance_matrix_ic(i,i);
        end

        error_transfer_matrix_vw = quat_to_euler_error_transfer_matrix(states(36),states(37),states(38),states(39));
        euler_covariance_matrix_vw = error_transfer_matrix_vw * covariance(36:39,36:39) * transpose(error_transfer_matrix_vw);
        for i=1:3
            output.euler_variances_vw(covIndex,i) = euler_covariance_matrix_vw(i,i);
        end
        
%         % Get most recent GPS data that had fallen behind the fusion time horizon
%         latest_gps_index = find((gps_data.time_us - 1e6 * param.fusion.gpsTimeDelay) < vicon_imu_data.time_us(imuIndex), 1, 'last' );
%         
%         if ~isempty(latest_gps_index)
%             % Check if GPS use is being blocked by the user
%             if ((local_time < param.control.gpsOnTime) && (local_time > param.control.gpsOffTime))
%                 gps_use_started = false;
%                 gps_use_blocked = true;
%             else
%                 gps_use_blocked = false;
%             end
%             
%             % If we haven't started using GPS, check that the quality is sufficient before aligning the position and velocity states to GPS
%             if (~gps_use_started && ~gps_use_blocked)
%                 if ((gps_data.spd_error(latest_gps_index) < param.control.gpsSpdErrLim) && (gps_data.pos_error(latest_gps_index) < param.control.gpsPosErrLim))
%                     states(5:7) = gps_data.vel_ned(latest_gps_index,:);
%                     states(8:9) = gps_data.pos_ned(latest_gps_index,1:2);
%                     gps_use_started = true;
%                     last_drift_constrain_time = local_time;
%                 end
%             end
%             
%             % Fuse GPS data when available if GPS use has started
%             if (gps_use_started && ~gps_use_blocked && (latest_gps_index > last_gps_index))
%                 last_gps_index = latest_gps_index;
%                 gps_fuse_index = gps_fuse_index + 1;
%                 last_drift_constrain_time = local_time;
%                 
%                 % fuse NED GPS velocity
%                 [states,covariance,velInnov,velInnovVar] = FuseVelocity(states,covariance,gps_data.vel_ned(latest_gps_index,:),param.fusion.gpsVelGate,gps_data.spd_error(latest_gps_index));
%                 
%                 % data logging
%                 output.innovations.vel_time_lapsed(gps_fuse_index) = local_time;
%                 output.innovations.vel_innov(gps_fuse_index,:) = velInnov';
%                 output.innovations.vel_innov_var(gps_fuse_index,:) = velInnovVar';
%                 
%                 % fuse NE GPS position
%                 [states,covariance,posInnov,posInnovVar] = FusePosition(states,covariance,gps_data.pos_ned(latest_gps_index,:),param.fusion.gpsPosGate,gps_data.pos_error(latest_gps_index));
%                 
%                 % data logging
%                 output.innovations.pos_time_lapsed(gps_fuse_index) = local_time;
%                 output.innovations.posInnov(gps_fuse_index,:) = posInnov';
%                 output.innovations.posInnovVar(gps_fuse_index,:) = posInnovVar';
%             else
%                 % Check if drift is being corrected by some form of aiding and if not, fuse in a zero position measurement at 5Hz to prevent states diverging
%                 if ((local_time - last_drift_constrain_time) > param.control.velDriftTimeLim)
%                     if ((local_time - last_synthetic_velocity_fusion_time) > 0.2)
%                         [states,covariance,~,~] = FusePosition(states,covariance,zeros(1,2),100.0,param.control.gpsPosErrLim);
%                         last_synthetic_velocity_fusion_time = local_time;
%                     end
%                 end
%             end
%         end
% 
%         % Fuse new Baro data that has fallen beind the fusion time horizon
%         latest_baro_index = find((baro_data.time_us - 1e6 * param.fusion.baroTimeDelay) < vicon_imu_data.time_us(imuIndex), 1, 'last' );
%         if (latest_baro_index > last_baro_index)
%             last_baro_index = latest_baro_index;
%             baro_fuse_index = baro_fuse_index + 1;
%             
%             % fuse baro height
%             [states,covariance,hgtInnov,hgtInnovVar] = FuseBaroHeight(states,covariance,baro_data.height(latest_baro_index),param.fusion.baroHgtGate,param.fusion.baroHgtNoise);
%             
%             % data logging
%             output.innovations.hgt_time_lapsed(baro_fuse_index) = local_time;
%             output.innovations.hgtInnov(baro_fuse_index) = hgtInnov;
%             output.innovations.hgtInnovVar(baro_fuse_index) = hgtInnovVar;
%         end
%         
%         % Fuse new mag data that has fallen behind the fusion time horizon
%         latest_mag_index = find((mag_data.time_us - 1e6 * param.fusion.magTimeDelay) < vicon_imu_data.time_us(imuIndex), 1, 'last' );
%         if (latest_mag_index > last_mag_index)
%             last_mag_index = latest_mag_index;
%             mag_fuse_index = mag_fuse_index + 1;
%             
%             % output magnetic field length to help with diagnostics
%             output.innovations.magLength(mag_fuse_index) = sqrt(dot(mag_data.field_ga(latest_mag_index,:),mag_data.field_ga(latest_mag_index,:)));
%             
%             % fuse magnetometer data
%             if (param.fusion.magFuseMethod == 0 || param.fusion.magFuseMethod == 1)
%                 [states,covariance,magInnov,magInnovVar] = FuseMagnetometer(states,covariance,mag_data.field_ga(latest_mag_index,:),param.fusion.magFieldGate, param.fusion.magFieldError^2);
%                 
%                 % data logging
%                 output.innovations.mag_time_lapsed(mag_fuse_index) = local_time;
%                 output.innovations.magInnov(mag_fuse_index,:) = magInnov;
%                 output.innovations.magInnovVar(mag_fuse_index,:) = magInnovVar;
%                 
%                 if (param.fusion.magFuseMethod == 1)
%                     % fuse in the local declination value
%                     [states, covariance] = FuseMagDeclination(states, covariance, param.fusion.magDeclDeg*deg2rad);
%                     
%                 end
%                 
%             elseif (param.fusion.magFuseMethod == 2)
%                 % fuse magnetomer data as a single magnetic heading measurement
%                 [states, covariance, hdgInnov, hdgInnovVar] = FuseMagHeading(states, covariance, mag_data.field_ga(latest_mag_index,:), param.fusion.magDeclDeg*deg2rad, param.fusion.magHdgGate, param.fusion.magHdgError^2);
%                 
%                 % log data
%                 output.innovations.mag_time_lapsed(mag_fuse_index) = local_time;
%                 output.innovations.hdgInnov(mag_fuse_index) = hdgInnov;
%                 output.innovations.hdgInnovVar(mag_fuse_index) = hdgInnovVar;
%                 
%             end
%             
%         end

        % Fuse new camera data that has fallen behind the fusion time horizon 
        latest_cam_index = find((vicon_pose_data.time_us - 1e9 * param.fusion.camTimeDelay) < vicon_imu_data.time_us(imuIndex), 1, 'last' );
        if (latest_cam_index > last_cam_index)
            last_cam_index = latest_cam_index;
            cam_fuse_index = cam_fuse_index + 1;

            measurements = [vicon_pose_data.q0(latest_cam_index);vicon_pose_data.q1(latest_cam_index);vicon_pose_data.q2(latest_cam_index);vicon_pose_data.q3(latest_cam_index);vicon_pose_data.x(latest_cam_index);vicon_pose_data.y(latest_cam_index);vicon_pose_data.z(latest_cam_index)];

            % fuse camera attitude data
            if (param.fusion.camIsAbsolute == 1)
%                 meas_quat_vc = [vicon_pose_data.q0(latest_cam_index);vicon_pose_data.q1(latest_cam_index);vicon_pose_data.q2(latest_cam_index);vicon_pose_data.q3(latest_cam_index)];
                [states,covariance,cam_attInnov,cam_posInnov,cam_poseInnovVar] = FuseCamAttitudeAbsolute(param,states,covariance,measurements(1:4,:),measurements(5:7,:),param.fusion.camPoseGate,param.fusion.camQuatMeasAbsNoise,param.fusion.camPosMeasAbsNoise);
                
                % data logging
                output.innovations.cam_pose_time_lapsed(cam_fuse_index) = local_time;
                output.innovations.cam_attInnov(cam_fuse_index,:) = cam_attInnov';
                output.innovations.cam_posInnov(cam_fuse_index,:) = cam_posInnov';
                output.innovations.cam_poseInnovVar(cam_fuse_index,:) = (diag(cam_poseInnovVar,0))';

                % fuse camera position data
%                 meas_pos_cv = [vicon_pose_data.x(latest_cam_index);vicon_pose_data.y(latest_cam_index);vicon_pose_data.z(latest_cam_index)];
%                 [states,covariance,cam_posInnov,cam_posInnovVar] = FuseCamPositionAbsolute(param,states,covariance,measurements(5:7,:),param.fusion.camPoseGate,param.fusion.camPosMeasAbsNoise);
%                 
%                 % data logging
%                 output.innovations.cam_pos_time_lapsed(cam_fuse_index) = local_time;
%                 output.innovations.cam_posInnov(cam_fuse_index,:) = cam_posInnov';
%                 output.innovations.cam_posInnovVar(cam_fuse_index,:) = (diag(cam_posInnovVar,0))';
            
            elseif (param.fusion.camIsAbsolute == 0)
                [states,covariance,cam_attInnov,cam_posInnov,cam_poseInnovVar] = FuseCamAttitudeRelative(param, states_old, states, covariance_old, covariance, F_old, F, measurements_old(1:4,:), measurements(1:4,:), measurements_old(5:7,:), measurements(5:7,:), param.fusion.camPoseGate, param.fusion.camQuatMeasRelNoise, param.fusion.camPosMeasRelNoise);

                % data logging
                output.innovations.cam_att_time_lapsed(cam_fuse_index) = local_time;
                output.innovations.cam_attInnov(cam_fuse_index,:) = cam_attInnov';
                output.innovations.cam_posInnov(cam_fuse_index,:) = cam_posInnov';
                output.innovations.cam_poseInnovVar(cam_fuse_index,:) = (diag(cam_poseInnovVar,0))';

                % fuse camera position data
%                 [states,covariance,cam_posInnov,cam_posInnovVar] = FuseCamPositionRelative(param, states_old, states, covariance_old, covariance, F_old, F, measurements_old(5:7,:), measurements(5:7,:), param.fusion.camPoseGate, param.fusion.camPosMeasRelNoise);
%                 
%                 % data logging
%                 output.innovations.cam_pos_time_lapsed(cam_fuse_index) = local_time;
%                 output.innovations.cam_posInnov(cam_fuse_index,:) = cam_posInnov';
%                 output.innovations.cam_posInnovVar(cam_fuse_index,:) = (diag(cam_posInnovVar,0))';

                states_old = states;
                covariance_old = covariance;
                F_old = F;
                measurements_old = measurements;
                
            end

            

        end

%         % Check if optical flow use is being blocked by the user
%         if ((local_time < param.control.flowOnTime) && (local_time > param.control.flowOffTime))
%             flow_use_blocked = true;
%         else
%             flow_use_blocked = false;
%         end
%         
%         % Attempt to use optical flow and range finder data if available and not blocked
%         if (flowDataPresent && ~flow_use_blocked)
%             
%             % Get latest range finder data and gate to remove dropouts
%             last_range_index = find((rng_data.time_us - 1e6 * param.fusion.rangeTimeDelay) < vicon_imu_data.time_us(imuIndex), 1, 'last' );
%             if (rng_data.dist(last_range_index) < param.fusion.rngValidMax)
%                 range = max(rng_data.dist(last_range_index) , param.fusion.rngValidMin);
%                 last_valid_range_time = local_time;
%             end
%             
%             % Fuse optical flow data that has fallen behind the fusion time horizon if we have a valid range measurement
%             latest_flow_index = find((flow_data.time_us - 1e6 * param.fusion.flowTimeDelay) < vicon_imu_data.time_us(imuIndex), 1, 'last' );
%             
%             if (~isempty(latest_flow_index) && (latest_flow_index > last_flow_index) && ((local_time - last_valid_range_time) < param.fusion.rngTimeout))
%                 last_flow_index = latest_flow_index;
%                 flow_fuse_index = flow_fuse_index + 1;
%                 last_drift_constrain_time = local_time;
%                 
%                 % fuse flow data
%                 flowRate = [flow_data.flowX(latest_flow_index);flow_data.flowY(latest_flow_index)];
%                 bodyRate = [flow_data.bodyX(latest_flow_index);flow_data.bodyY(latest_flow_index)];
%                 [states,covariance,flowInnov,flowInnovVar] = FuseOpticalFlow(states, covariance, flowRate, bodyRate, range, param.fusion.flowRateError^2, param.fusion.flowGate);
%                 
%                 % data logging
%                 output.innovations.flow_time_lapsed(flow_fuse_index) = local_time;
%                 output.innovations.flowInnov(flow_fuse_index,:) = flowInnov;
%                 output.innovations.flowInnovVar(flow_fuse_index,:) = flowInnovVar;
%                 
%             end
%             
%         end
%         
%         % Check if optical flow use is being blocked by the user
%         if ((local_time < param.control.visoOnTime) && (local_time > param.control.visoOffTime))
%             viso_use_blocked = true;
%         else
%             viso_use_blocked = false;
%         end
%         
%         % attempt to use ZED camera visual odmetry data if available and not blocked
%         if (visoDataPresent && ~viso_use_blocked)
%             
%             % Fuse ZED camera body frame odmometry data that has fallen behind the fusion time horizon
%             latest_viso_index = find((viso_data.time_us - 1e6 * param.fusion.bodyVelTimeDelay) < vicon_imu_data.time_us(imuIndex), 1, 'last' );
%             if (latest_viso_index > last_viso_index)
%                 last_viso_index = latest_viso_index;
%                 viso_fuse_index = viso_fuse_index + 1;
%                 last_drift_constrain_time = local_time;
%                 
%                 % convert delta positon measurements to velocity
%                 relVelBodyMea = [viso_data.dPosX(latest_viso_index);viso_data.dPosY(latest_viso_index);viso_data.dPosZ(latest_viso_index)]./viso_data.dt(latest_viso_index);
%                 
%                 % convert quality metric to equivalent observation error
%                 % (this is a guess)
%                 quality = viso_data.qual(latest_viso_index);
%                 bodyVelError = param.fusion.bodyVelErrorMin * quality + param.fusion.bodyVelErrorMax * (1 - quality);
%                 
%                 % fuse measurements
%                 [states,covariance,bodyVelInnov,bodyVelInnovVar] = FuseBodyVel(states, covariance, relVelBodyMea, bodyVelError^2, param.fusion.bodyVelGate);
%                 
%                 % data logging
%                 output.innovations.bodyVel_time_lapsed(viso_fuse_index) = local_time;
%                 output.innovations.bodyVelInnov(viso_fuse_index,:) = bodyVelInnov;
%                 output.innovations.bodyVelInnovVar(viso_fuse_index,:) = bodyVelInnovVar;
%                 
%             end
%             
%         end
    
    % update average delta time estimate
    output.dt = dtCovInt / covIndex;
    
end

end