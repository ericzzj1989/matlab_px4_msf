clear all;
close all;

% add required paths
addpath('../Common');

% load test data
load '../TestData/PX4-MSF/baro_data.mat';
load '../TestData/PX4-MSF/gps_data.mat';
load '../TestData/PX4-MSF/vicon_imu_data.mat';
load '../TestData/PX4-MSF/vicon_pose_data.mat';
load '../TestData/PX4-MSF/mag_data.mat';

% accel_tmp = vicon_imu_data.accel_x;
% vicon_imu_data.accel_x = vicon_imu_data.accel_y;
% vicon_imu_data.accel_y = accel_tmp;
% vicon_imu_data.accel_z = -vicon_imu_data.accel_z;

% set parameters to default values
run('SetParameters.m');

% run the filter replay
output = RunFilter(param,vicon_imu_data,mag_data,baro_data,gps_data,vicon_pose_data);


% save output

% absolute output
if (param.fusion.camIsAbsolute == 1)
    % generate and save output plots
    runIdentifier = ' : PX4-MSF-ABSOLUTE data replay ';
    folder = strcat('../OutputPlots_VICON/PX4-MSF-ABSOLUTE');
    PlotData(output,folder,runIdentifier);

    % save output data
    folder = '../OutputData_VICON/PX4-MSF-ABSOLUTE';
    fileName = '../OutputData_VICON/PX4-MSF-ABSOLUTE/ekf_msf_absolute_replay_output.mat';
    if ~exist(folder,'dir')
        mkdir(folder);
    end
    save(fileName,'output');

% relative output
elseif (param.fusion.camIsAbsolute == 0)
    
    runIdentifier = ' : PX4-MSF-RELATIVE data replay ';
    folder = strcat('../OutputPlots_VICON/PX4-MSF-RELATIVE');
    PlotData(output,folder,runIdentifier);




%     rad2deg = 180/pi;
%     if ~exist(folder,'dir')
%         mkdir(folder);
%     end
%     plotDimensions = [0 0 210*3 297*3];


%     %% plot position from camera to imu frame estimates
%     figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', 'Position ci');
%     h=gcf;
%     set(h,'PaperOrientation','portrait');
%     set(h,'PaperUnits','normalized');
%     set(h,'PaperPosition', [0 0 1 1]);
% 
%     subplot(3,1,1);
%     plot(output.time_lapsed,output.position_ci(:,1));
%     grid on;
%     titleText=strcat({'Position from Camera to Imu Frame Estimates'},runIdentifier);
%     title(titleText);
%     ylabel('X ci (m)');
%     xlabel('time (sec)');
% 
%     subplot(3,1,2);
%     plot(output.time_lapsed,output.position_ci(:,2));
%     grid on;
%     ylabel('Y ci (m)');
%     xlabel('time (sec)');
% 
%     subplot(3,1,3);
%     plot(output.time_lapsed,output.position_ci(:,3));
%     grid on;
%     ylabel('Z ci (m)');
%     xlabel('time (sec)');
%     
%     fileName='position_ci_estimates.png';
%     fullFileName = fullfile(folder, fileName);
%     saveas(h,fullFileName);
% 
%     %% plot overall position from camera to imu frame estimates
%     figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', 'Overall Position ci');
%     h=gcf;
%     set(h,'PaperOrientation','portrait');
%     set(h,'PaperUnits','normalized');
%     set(h,'PaperPosition', [0 0 1 1]);
% 
%     plot(output.time_lapsed,output.position_ci(:,1),'r');
%     hold on;
%     grid on;
%     titleText=strcat({'Overall Position from Camera to Imu Frame Estimates'},runIdentifier);
%     title(titleText);
%     plot(output.time_lapsed,output.position_ci(:,2),'b');
%     hold on;
%     plot(output.time_lapsed,output.position_ci(:,3),'g');
%     xlabel('time (sec)');
%     legend('X','Y','Z');
% 
%     fileName='overall_position_ci_estimates.png';
%     fullFileName = fullfile(folder, fileName);
%     saveas(h,fullFileName);
% 
% 
%     %% plot 3d position from camera to imu frame estimates
%     figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', '3D Position ci');
%     h=gcf;
%     set(h,'PaperOrientation','portrait');
%     set(h,'PaperUnits','normalized');
%     set(h,'PaperPosition', [0 0 1 1]);
% 
%     plot3(output.position_ci(:,1),output.position_ci(:,2),output.position_ci(:,3));
%     grid on;
%     titleText=strcat({'3D Position from Camera to Imu Frame Estimates'},runIdentifier);
%     title(titleText);
%     zlabel('Z ci (m)');
%     ylabel('Y ci (m)');
%     xlabel('X ci (m)');
% 
%     fileName='3d_position_ci_estimates.png';
%     fullFileName = fullfile(folder, fileName);
%     saveas(h,fullFileName);



%     %% plot euler angle from imu to camera frame estimates
%     figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', 'Euler Angle vw');
%     h=gcf;
%     set(h,'PaperOrientation','portrait');
%     set(h,'PaperUnits','normalized');
%     set(h,'PaperPosition', [0 0 1 1]);
% 
%     margin = 0;
% 
%     subplot(3,1,1);
%     plot(output.time_lapsed,output.euler_angles_vw(:,1)*rad2deg);
%     minVal = rad2deg*min(output.euler_angles_vw(:,1))-margin;
%     maxVal = rad2deg*max(output.euler_angles_vw(:,1))+margin;
%     ylim([minVal maxVal]);
%     grid on;
%     titleText=strcat({'Euler Angle from Vision to World Frame Estimates'},runIdentifier);
%     title(titleText);
%     ylabel('Roll vw (deg)');
%     xlabel('time (sec)');
% 
%     subplot(3,1,2);
%     plot(output.time_lapsed,output.euler_angles_vw(:,2)*rad2deg);
%     minVal = rad2deg*min(output.euler_angles_vw(:,2))-margin;
%     maxVal = rad2deg*max(output.euler_angles_vw(:,2))+margin;
%     ylim([minVal maxVal]);
%     grid on;
%     ylabel('Pitch vw (deg)');
%     xlabel('time (sec)');
% 
%     subplot(3,1,3);
%     plot(output.time_lapsed,output.euler_angles_vw(:,3)*rad2deg);
%     minVal = rad2deg*min(output.euler_angles_vw(:,3))-margin;
%     maxVal = rad2deg*max(output.euler_angles_vw(:,3))+margin;
%     ylim([minVal maxVal]);
%     grid on;
%     ylabel('Yaw vw (deg)');
%     xlabel('time (sec)');
% 
%     fileName='euler_angle_vw_estimates.png';
%     fullFileName = fullfile(folder, fileName);
%     saveas(h,fullFileName);


%     figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', 'Lamda');
%     h=gcf;
%     set(h,'PaperOrientation','portrait');
%     set(h,'PaperUnits','normalized');
%     set(h,'PaperPosition', [0 0 1 1]);
% 
%     plot(output.time_lapsed,output.lamda);
%     grid on;
%     titleText=strcat({'Camera Scale Estimates'},runIdentifier);
%     title(titleText);
%     xlabel('time (sec)');
% 
%     fileName='lamda_estimates.png';
%     fullFileName = fullfile(folder, fileName);
%     saveas(h,fullFileName);




    % save output data
    folder = '../OutputData_VICON/PX4-MSF-RELATIVE';
    fileName = '../OutputData_VICON/PX4-MSF-RELATIVE/ekf_msf_relative_replay_output.mat';
    if ~exist(folder,'dir')
        mkdir(folder);
    end
    save(fileName,'output');
end


%figure euler
% figure;plot(output.time_lapsed,output.euler_angles_wi(:,1),'b');grid on;
% hold on;plot(output.time_lapsed,output.euler_angles_vc(:,1),'r');title('eular(1)');
% 
% figure;plot(output.time_lapsed,output.euler_angles_wi(:,2),'b');grid on;
% hold on;plot(output.time_lapsed,output.euler_angles_vc(:,2),'r');title('eular(2)');
% 
% figure;plot(output.time_lapsed,output.euler_angles_wi(:,3),'b');grid on;
% hold on;plot(output.time_lapsed,output.euler_angles_vc(:,3),'r');title('eular(3)');
% 
% %figure position
% figure;plot(output.time_lapsed,output.position_NED_iw(:,1),'b');grid on;
% hold on;plot(output.time_lapsed,output.position_cv(:,1),'r');title('position(x)');
% 
% figure;plot(output.time_lapsed,output.position_NED_iw(:,2),'b');grid on;
% hold on;plot(output.time_lapsed,output.position_cv(:,2),'r');title('position(y)');
% 
% figure;plot(output.time_lapsed,output.position_NED_iw(:,3),'b');grid on;
% hold on;plot(output.time_lapsed,output.position_cv(:,3),'r');title('position(z)');

% %figure lamda
% figure;plot(output.time_lapsed,output.lamda,'b');grid on;title('lamda');
% 
% %figure pvw
% figure;plot(output.time_lapsed,output.position_vw,'b');grid on;title('position(vw)');
% %figure qvw
% figure;plot(output.time_lapsed,output.euler_angles_vw,'b');grid on;title('euler(vw)');
% 
% %figure pci
% figure;plot(output.time_lapsed,output.position_ci,'b');grid on;title('position(ci)');
% %figure qci
% figure;plot(output.time_lapsed,output.euler_angles_ic,'b');grid on;title('euler(ci)');
% %figure acc_bias
% figure;plot(output.time_lapsed,output.accel_bias(:,3),'b');grid on;title('accel_bias');
% %figure bias_bias
% figure;plot(output.time_lapsed,output.gyro_bias,'b');grid on;title('gyro_bias');
% 
% figure;plot3(output.position_NED_iw(:,1),output.position_NED_iw(:,2),output.position_NED_iw(:,3),'b');grid on;
% hold on;plot3(output.position_cv(:,1),output.position_cv(:,2),output.position_cv(:,3),'r');




% For Debug
% 
% figure;plot(output.innovations.cam_att_time_lapsed,output.innovations.cam_attInnov(:,1),'b');grid on;title('cam_attInnov(1)');
% figure;plot(output.innovations.cam_att_time_lapsed,output.innovations.cam_attInnov(:,2),'b');grid on;title('cam_attInnov(2)');
% figure;plot(output.innovations.cam_att_time_lapsed,output.innovations.cam_attInnov(:,3),'b');grid on;title('cam_attInnov(3)');
% figure;plot(output.innovations.cam_att_time_lapsed,output.innovations.cam_attInnov(:,4),'b');grid on;title('cam_attInnov(4)');
% 
% figure;plot(output.innovations.cam_att_time_lapsed,output.innovations.cam_attInnov,'b');grid on;title('cam_attInnov(1)');
% 
% figure;plot(vicon_pose_data.time_us/1e9,vicon_pose_data.q0,'r');grid on;title('vicon_pose_data(0)');
% hold on;plot(output.time_lapsed,output.quat(:,1),'b');grid on;title('output.quat0');
% 
% figure;plot(vicon_pose_data.time_us/1e9,vicon_pose_data.q1,'r');grid on;title('vicon_pose_data(1)');
% hold on;plot(output.time_lapsed,output.quat(:,2),'b');grid on;title('output.quat1');
% 
% figure;plot(vicon_pose_data.time_us/1e9,vicon_pose_data.q2,'r');grid on;title('vicon_pose_data(2)');
% hold on;plot(output.time_lapsed,output.quat(:,3),'b');grid on;title('output.quat2');
% 
% figure;plot(vicon_pose_data.time_us/1e9,vicon_pose_data.q3,'r');grid on;title('vicon_pose_data(3)');
% hold on;plot(output.time_lapsed,output.quat(:,4),'b');grid on;title('output.quat3');



% figure;plot(output.innovations.cam_pos_time_lapsed,output.innovations.cam_posInnov(:,1),'b');grid on;title('cam_posInnov(1)');
% figure;plot(output.innovations.cam_pos_time_lapsed,output.innovations.cam_posInnov(:,2),'b');grid on;title('cam_posInnov(2)');
% figure;plot(output.innovations.cam_pos_time_lapsed,output.innovations.cam_posInnov(:,3),'b');grid on;title('cam_posInnov(3)');



% figure;plot(output.time_lapsed,output.velocity_NED(:,3))
% figure;plot(output.time_lapsed,output.state_variances(:,9));
% figure;plot(output.time_lapsed,output.state_variances(:,10));
% close all;
% figure;plot(diff(output.pvcposi(:,3)),'r');
% figure;plot(vicon_imu_data.accel_z)




