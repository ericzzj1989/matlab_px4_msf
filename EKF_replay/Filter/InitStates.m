function [states, imu_start_index, pose_start_index] = InitStates(param,vicon_imu_data,gps_data,mag_data,baro_data,vicon_pose_data)

% constants
deg2rad = pi/180;

% initialise the state vector and quaternion
% new state number is 39, adding 15 states related to camera
states = zeros(39,1);
quat = [1;0;0;0];

if (param.control.waitForGps == 1)
    % find IMU start index that coresponds to first valid GPS data
    imu_start_index = (find(vicon_imu_data.time_us > pose_data.time_us, 1, 'first' ) - 50);
    imu_start_index = max(imu_start_index,1);
else
    imu_start_index = 1;
end

% average first 100 accel readings to reduce effect of vibration
initAccel(1) = mean(vicon_imu_data.accel_x(imu_start_index:imu_start_index+99));
initAccel(2) = mean(vicon_imu_data.accel_y(imu_start_index:imu_start_index+99));
initAccel(3) = mean(vicon_imu_data.accel_z(imu_start_index:imu_start_index+99));

% align tilt using gravity vector (If the velocity is changing this will
% induce errors)
% quat = AlignTilt(quat,initAccel);
states(1:4) = [1;0;0;0];

% % add a roll, pitch, yaw mislignment
% quat_align_err = EulToQuat([param.control.rollAlignErr,param.control.pitchAlignErr,param.control.yawAlignErr]);
% quat = QuatMult(quat,quat_align_err);
% 
% % find magnetometer start index
% mag_start_index = (find(mag_data.time_us > vicon_imu_data.time_us(imu_start_index), 1, 'first' ) - 5);
% mag_start_index = max(mag_start_index,1);
% 
% % mean to reduce effect of noise in data
% magBody(1,1) = mean(mag_data.field_ga(mag_start_index:mag_start_index+9,1));
% magBody(2,1) = mean(mag_data.field_ga(mag_start_index:mag_start_index+9,2));
% magBody(3,1) = mean(mag_data.field_ga(mag_start_index:mag_start_index+9,3));
% 
% % align heading and initialise the NED magnetic field states
% quat = AlignHeading(quat,magBody,param.fusion.magDeclDeg*deg2rad);
% states(1:4) = quat;
% 
% % initialise the NED magnetic field states
% Tbn = Quat2Tbn(quat);
% states(17:19) = Tbn*magBody;
% 
% if (param.control.waitForGps == 1)
%     % initialise velocity and position using gps
%     states(5:7) = gps_data.vel_ned(gps_data.start_index,:);
%     states(8:9) = gps_data.pos_ned(gps_data.start_index,1:2);
% else
%     % initialise to be stationary at the origin
%     states(5:7) = zeros(1,3);
%     states(8:9) = zeros(1,2);
% end
% 
% % find baro start index
% baro_start_index = (find(baro_data.time_us > vicon_imu_data.time_us(imu_start_index), 1, 'first' ) - 10);
% baro_start_index = max(baro_start_index,1);
% 
% % average baro data and initialise the vertical position
% states(10) = -mean(baro_data.height(baro_start_index:baro_start_index+20));

% find camera pose index
pose_start_index = (find(vicon_pose_data.time_us > vicon_imu_data.time_us(imu_start_index), 1, 'first') - 10);
pose_start_index = max(pose_start_index,1);

% TO DO: initialise camera pose states
quat_ic = [1;0;0;0];
states(25) = 1; %vicon_pose_data.lamda(pose_start_index,:);
states(29:32) = quat_ic; % pose_data.quat_ic(pose_start_index,:);
quat_vc = [vicon_pose_data.q0(pose_start_index),vicon_pose_data.q1(pose_start_index),vicon_pose_data.q2(pose_start_index),vicon_pose_data.q3(pose_start_index)];
quat_vw = QuatMult(quat_vc', [states(1);-states(2);-states(3);-states(4)]);
states(36:39) = quat_vw;

pos_cv = [vicon_pose_data.x(pose_start_index),vicon_pose_data.y(pose_start_index),vicon_pose_data.z(pose_start_index)];
pos_iw = (inv(Quat2Tbn(quat_vw))) * pos_cv';
states(8:10) = pos_iw;

end