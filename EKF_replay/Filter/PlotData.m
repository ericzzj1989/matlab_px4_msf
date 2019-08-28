function PlotData(output,folder,runIdentifier)
rad2deg = 180/pi;
if ~exist(folder,'dir')
    mkdir(folder);
end
plotDimensions = [0 0 210*3 297*3];

%% plot euler angle estimates
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', 'Euler Angle wi');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

margin = 5;

subplot(3,1,1);
plot(output.time_lapsed,[output.euler_angles_wi(:,1)*rad2deg,output.euler_angles_wi(:,1)*rad2deg-2*sqrt(output.euler_variances_wi(:,1)*rad2deg),output.euler_angles_wi(:,1)*rad2deg+2*sqrt(output.euler_variances_wi(:,1)*rad2deg)]);
minVal = rad2deg*min(output.euler_angles_wi(:,1))-margin;
maxVal = rad2deg*max(output.euler_angles_wi(:,1))+margin;
ylim([minVal maxVal]);
grid on;
titleText=strcat({'Euler Angle from World to Imu Frame Estimates'},runIdentifier);
title(titleText);
ylabel('Roll wi (deg)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

subplot(3,1,2);
plot(output.time_lapsed,[output.euler_angles_wi(:,2)*rad2deg,output.euler_angles_wi(:,2)*rad2deg-2*sqrt(output.euler_variances_wi(:,2)*rad2deg),output.euler_angles_wi(:,2)*rad2deg+2*sqrt(output.euler_variances_wi(:,2)*rad2deg)]);
minVal = rad2deg*min(output.euler_angles_wi(:,2))-margin;
maxVal = rad2deg*max(output.euler_angles_wi(:,2))+margin;
ylim([minVal maxVal]);
grid on;
ylabel('Pitch wi (deg)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

subplot(3,1,3);
plot(output.time_lapsed,[output.euler_angles_wi(:,3)*rad2deg,output.euler_angles_wi(:,3)*rad2deg-2*sqrt(output.euler_variances_wi(:,3)*rad2deg),output.euler_angles_wi(:,3)*rad2deg+2*sqrt(output.euler_variances_wi(:,3)*rad2deg)]);
minVal = rad2deg*min(output.euler_angles_wi(:,3))-margin;
maxVal = rad2deg*max(output.euler_angles_wi(:,3))+margin;
ylim([minVal maxVal]);
grid on;
ylabel('Yaw wi (deg)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

fileName='euler_angle_wi_estimates.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

% %% plot NED velocity estimates
% figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
% h=gcf;
% set(h,'PaperOrientation','portrait');
% set(h,'PaperUnits','normalized');
% set(h,'PaperPosition', [0 0 1 1]);
% 
% subplot(3,1,1);
% plot(output.time_lapsed,[output.velocity_NED(:,1),output.velocity_NED(:,1)+2*sqrt(output.state_variances(:,5)),output.velocity_NED(:,1)-2*sqrt(output.state_variances(:,5))]);
% grid on;
% titleText=strcat({'NED Velocity Estimates'},runIdentifier);
% title(titleText);
% ylabel('North (m/s)');
% xlabel('time (sec)');
% legend('estimate','upper 95% bound','lower 95% bound');
% 
% subplot(3,1,2);
% plot(output.time_lapsed,[output.velocity_NED(:,2),output.velocity_NED(:,2)+2*sqrt(output.state_variances(:,6)),output.velocity_NED(:,2)-2*sqrt(output.state_variances(:,6))]);
% grid on;
% ylabel('East (m/s)');
% xlabel('time (sec)');
% legend('estimate','upper 95% bound','lower 95% bound');
% 
% subplot(3,1,3);
% plot(output.time_lapsed,[output.velocity_NED(:,3),output.velocity_NED(:,3)+2*sqrt(output.state_variances(:,7)),output.velocity_NED(:,3)-2*sqrt(output.state_variances(:,7))]);
% grid on;
% ylabel('Down (m/s)');
% xlabel('time (sec)');
% legend('estimate','upper 95% bound','lower 95% bound');
% 
% fileName='velocity_estimates.png';
% fullFileName = fullfile(folder, fileName);
% saveas(h,fullFileName);

%% plot NED position estimates
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', 'NED Position');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

subplot(3,1,1);
plot(output.time_lapsed,[output.position_NED_iw(:,1),output.position_NED_iw(:,1)+2*sqrt(output.state_variances(:,8)),output.position_NED_iw(:,1)-2*sqrt(output.state_variances(:,8))]);
grid on;
titleText=strcat({'NED Position from Imu to World Frame Estimates'},runIdentifier);
title(titleText);
ylabel('North (m)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

subplot(3,1,2);
plot(output.time_lapsed,[output.position_NED_iw(:,2),output.position_NED_iw(:,2)+2*sqrt(output.state_variances(:,9)),output.position_NED_iw(:,2)-2*sqrt(output.state_variances(:,9))]);
grid on;
ylabel('East (m)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

subplot(3,1,3);
plot(output.time_lapsed,[output.position_NED_iw(:,3),output.position_NED_iw(:,3)+2*sqrt(output.state_variances(:,10)),output.position_NED_iw(:,3)-2*sqrt(output.state_variances(:,10))]);
grid on;
ylabel('Down (m)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

fileName='position_iw_estimates.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

%% plot 3D NED position estimates
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', '3D NED Position');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

plot3(output.position_NED_iw(:,1),output.position_NED_iw(:,2),output.position_NED_iw(:,3));
grid on;
titleText=strcat({'3D NED Position from Imu to World Frame Estimates'},runIdentifier);
title(titleText);
zlabel('Down (m)');
ylabel('East (m)');
xlabel('North (m)');

fileName='3d_position_iw_estimates.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

%% plot IMU gyro bias estimates
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', 'IMU Gyro Bias');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

margin = 0.1;

subplot(3,1,1);
plot(output.time_lapsed,(1/output.dt)*[output.gyro_bias(:,1),output.gyro_bias(:,1)+2*sqrt(output.state_variances(:,11)),output.gyro_bias(:,1)-2*sqrt(output.state_variances(:,11))]*rad2deg);%%output.gyro_bias(:,1)*rad2deg);
minVal = (1/output.dt)*rad2deg*min(output.gyro_bias(:,1))-margin;
maxVal = (1/output.dt)*rad2deg*max(output.gyro_bias(:,1))+margin;
ylim([minVal maxVal]);
grid on;
titleText=strcat({'IMU Gyro Bias Estimates'},runIdentifier);
title(titleText);
ylabel('X gyro (deg/s)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

subplot(3,1,2);
plot(output.time_lapsed,(1/output.dt)*[output.gyro_bias(:,2),output.gyro_bias(:,2)+2*sqrt(output.state_variances(:,12)),output.gyro_bias(:,2)-2*sqrt(output.state_variances(:,12))]*rad2deg);
minVal = (1/output.dt)*rad2deg*min(output.gyro_bias(:,2))-margin;
maxVal = (1/output.dt)*rad2deg*max(output.gyro_bias(:,2))+margin;
ylim([minVal maxVal]);
grid on;
ylabel('Y gyro (deg/s)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

subplot(3,1,3);
plot(output.time_lapsed,(1/output.dt)*[output.gyro_bias(:,3),output.gyro_bias(:,3)+2*sqrt(output.state_variances(:,13)),output.gyro_bias(:,3)-2*sqrt(output.state_variances(:,13))]*rad2deg);
minVal = (1/output.dt)*rad2deg*min(output.gyro_bias(:,3))-margin;
maxVal = (1/output.dt)*rad2deg*max(output.gyro_bias(:,3))+margin;
ylim([minVal maxVal]);
grid on;
ylabel('Z gyro (deg/s)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

fileName='imu_gyro_bias_estimates.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

%% plot IMU accel bias estimates
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', 'IMU Accel Bias');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

margin = 0.1;

subplot(3,1,1);
plot(output.time_lapsed,(1/output.dt)*[output.accel_bias(:,1),output.accel_bias(:,1)+2*sqrt(output.state_variances(:,14)),output.accel_bias(:,1)-2*sqrt(output.state_variances(:,14))]);
titleText=strcat({'IMU Accel Bias Estimates'},runIdentifier);
title(titleText);
minVal = (1/output.dt)*min(output.accel_bias(:,1))-margin;
maxVal = (1/output.dt)*max(output.accel_bias(:,1))+margin;
ylim([minVal maxVal]);
grid on;
ylabel('X accel (m/s/s)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

subplot(3,1,2);
plot(output.time_lapsed,(1/output.dt)*[output.accel_bias(:,2),output.accel_bias(:,2)+2*sqrt(output.state_variances(:,15)),output.accel_bias(:,2)-2*sqrt(output.state_variances(:,15))]);
minVal = (1/output.dt)*min(output.accel_bias(:,1))-margin;
maxVal = (1/output.dt)*max(output.accel_bias(:,1))+margin;
ylim([minVal maxVal]);
grid on;
ylabel('Y accel (m/s/s)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

subplot(3,1,3);
plot(output.time_lapsed,(1/output.dt)*[output.accel_bias(:,3),output.accel_bias(:,3)+2*sqrt(output.state_variances(:,16)),output.accel_bias(:,3)-2*sqrt(output.state_variances(:,16))]);
minVal = (1/output.dt)*min(output.accel_bias(:,1))-margin;
maxVal = (1/output.dt)*max(output.accel_bias(:,1))+margin;
ylim([minVal maxVal]);
grid on;
ylabel('Z accel (m/s/s)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

fileName='imu_accel_bias_estimates.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);
 
% %% plot magnetometer bias estimates
% if (output.magFuseMethod <= 1)
%     figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
%     h=gcf;
%     set(h,'PaperOrientation','portrait');
%     set(h,'PaperUnits','normalized');
%     set(h,'PaperPosition', [0 0 1 1]);
%     
%     subplot(3,1,1);
%     plot(output.time_lapsed',[output.mag_XYZ(:,1),output.mag_XYZ(:,1)+2*sqrt(output.state_variances(:,20)),output.mag_XYZ(:,1)-2*sqrt(output.state_variances(:,20))]);
%     grid on;
%     titleText=strcat({'Magnetometer Bias Estimates'},runIdentifier);
%     title(titleText);
%     ylabel('X bias (gauss)');
%     xlabel('time (sec)');
%     legend('estimate','upper 95% bound','lower 95% bound');
%     
%     subplot(3,1,2);
%     plot(output.time_lapsed',[output.mag_XYZ(:,2),output.mag_XYZ(:,2)+2*sqrt(output.state_variances(:,21)),output.mag_XYZ(:,2)-2*sqrt(output.state_variances(:,21))]);
%     grid on;
%     ylabel('Y bias (gauss)');
%     xlabel('time (sec)');
%     legend('estimate','upper 95% bound','lower 95% bound');
%     
%     subplot(3,1,3);
%     plot(output.time_lapsed',[output.mag_XYZ(:,3),output.mag_XYZ(:,3)+2*sqrt(output.state_variances(:,22)),output.mag_XYZ(:,3)-2*sqrt(output.state_variances(:,22))]);
%     grid on;
%     ylabel('Z bias (gauss)');
%     xlabel('time (sec)');
%     legend('estimate','upper 95% bound','lower 95% bound');
%     
%     fileName='body_field_estimates.png';
%     fullFileName = fullfile(folder, fileName);
%     saveas(h,fullFileName);
% end
% 
% %% plot earth field estimates
% if (output.magFuseMethod <= 1)
%     figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
%     h=gcf;
%     set(h,'PaperOrientation','portrait');
%     set(h,'PaperUnits','normalized');
%     set(h,'PaperPosition', [0 0 1 1]);
%     
%     margin = 0.1;
%     
%     subplot(4,1,1);
%     plot(output.time_lapsed',[output.mag_NED(:,1),output.mag_NED(:,1)+2*sqrt(output.state_variances(:,17)),output.mag_NED(:,1)-2*sqrt(output.state_variances(:,17))]);
%     minVal = min(output.mag_NED(:,1))-margin;
%     maxVal = max(output.mag_NED(:,1))+margin;
%     ylim([minVal maxVal]);
%     grid on;
%     titleText=strcat({'Earth Magnetic Field Estimates'},runIdentifier);
%     title(titleText);
%     ylabel('North (gauss)');
%     xlabel('time (sec)');
%     legend('estimate','upper 95% bound','lower 95% bound');
%     
%     subplot(4,1,2);
%     plot(output.time_lapsed',[output.mag_NED(:,2),output.mag_NED(:,2)+2*sqrt(output.state_variances(:,18)),output.mag_NED(:,2)-2*sqrt(output.state_variances(:,18))]);
%     minVal = min(output.mag_NED(:,2))-margin;
%     maxVal = max(output.mag_NED(:,2))+margin;
%     ylim([minVal maxVal]);
%     grid on;
%     ylabel('East (gauss)');
%     xlabel('time (sec)');
%     legend('estimate','upper 95% bound','lower 95% bound');
%     
%     subplot(4,1,3);
%     plot(output.time_lapsed',[output.mag_NED(:,3),output.mag_NED(:,3)+2*sqrt(output.state_variances(:,19)),output.mag_NED(:,3)-2*sqrt(output.state_variances(:,19))]);
%     grid on;
%     minVal = min(output.mag_NED(:,3))-margin;
%     maxVal = max(output.mag_NED(:,3))+margin;
%     ylim([minVal maxVal]);
%     ylabel('Down (gauss)');
%     xlabel('time (sec)');
%     legend('estimate','upper 95% bound','lower 95% bound');
%     
%     subplot(4,1,4);
%     plot(output.time_lapsed',rad2deg*atan2(output.mag_NED(:,2),output.mag_NED(:,1)));
%     grid on;
%     titleText=strcat({'Magnetic Declination Estimate'},runIdentifier);
%     title(titleText);
%     ylabel('declination (deg)');
%     xlabel('time (sec)');
%     
%     fileName='earth_field_estimates.png';
%     fullFileName = fullfile(folder, fileName);
%     saveas(h,fullFileName);
% end

%% plot vo states estimates

% lamda
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', 'Lamda');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

plot(output.time_lapsed,[output.lamda,output.lamda+2*sqrt(output.state_variances(:,25)),output.lamda-2*sqrt(output.state_variances(:,25))]);
grid on;
titleText=strcat({'Camera Scale Estimates'},runIdentifier);
title(titleText);
ylabel('Lamda');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

fileName='lamda_estimates.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

%% plot position from camera to imu frame estimates
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', 'Position ci');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

subplot(3,1,1);
plot(output.time_lapsed,[output.position_ci(:,1),output.position_ci(:,1)+2*sqrt(output.state_variances(:,26)),output.position_ci(:,1)-2*sqrt(output.state_variances(:,26))]);
grid on;
titleText=strcat({'Position from Camera to Imu Frame Estimates'},runIdentifier);
title(titleText);
ylabel('X ci (m)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

subplot(3,1,2);
plot(output.time_lapsed,[output.position_ci(:,2),output.position_ci(:,2)+2*sqrt(output.state_variances(:,27)),output.position_ci(:,2)-2*sqrt(output.state_variances(:,27))]);
grid on;
ylabel('Y ci (m)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

subplot(3,1,3);
plot(output.time_lapsed,[output.position_ci(:,3),output.position_ci(:,3)+2*sqrt(output.state_variances(:,28)),output.position_ci(:,3)-2*sqrt(output.state_variances(:,28))]);
grid on;
ylabel('Z ci (m)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

fileName='position_ci_estimates.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

%% plot 3d position from camera to imu frame estimates
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', '3D Position ci');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

plot3(output.position_ci(:,1),output.position_ci(:,2),output.position_ci(:,3));
grid on;
titleText=strcat({'3D NED Position from Camera to Imu Frame Estimates'},runIdentifier);
title(titleText);
zlabel('Z ci (m)');
ylabel('Y ci (m)');
xlabel('X ci (m)');

fileName='3d_position_ci_estimates.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

%% plot euler angle from imu to camera frame estimates
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', 'Euler Angle ic');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

% margin = 0;

subplot(3,1,1);
plot(output.time_lapsed,[output.euler_angles_ic(:,1)*rad2deg,output.euler_angles_ic(:,1)*rad2deg-2*sqrt(output.euler_variances_ic(:,1)*rad2deg),output.euler_angles_ic(:,1)*rad2deg+2*sqrt(output.euler_variances_ic(:,1)*rad2deg)]);
% minVal = rad2deg*min(output.euler_angles_ic(:,1))-margin;
% maxVal = rad2deg*max(output.euler_angles_ic(:,1))+margin;
% ylim([minVal maxVal]);
grid on;
titleText=strcat({'Euler Angle from Imu to Camera Frame Estimates'},runIdentifier);
title(titleText);
ylabel('Roll ic (deg)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

subplot(3,1,2);
plot(output.time_lapsed,[output.euler_angles_ic(:,2)*rad2deg,output.euler_angles_ic(:,2)*rad2deg-2*sqrt(output.euler_variances_ic(:,2)*rad2deg),output.euler_angles_ic(:,2)*rad2deg+2*sqrt(output.euler_variances_ic(:,2)*rad2deg)]);
% minVal = rad2deg*min(output.euler_angles_ic(:,2))-margin;
% maxVal = rad2deg*max(output.euler_angles_ic(:,2))+margin;
% ylim([minVal maxVal]);
grid on;
ylabel('Pitch ic (deg)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

subplot(3,1,3);
plot(output.time_lapsed,[output.euler_angles_ic(:,3)*rad2deg,output.euler_angles_ic(:,3)*rad2deg-2*sqrt(output.euler_variances_ic(:,3)*rad2deg),output.euler_angles_ic(:,3)*rad2deg+2*sqrt(output.euler_variances_ic(:,3)*rad2deg)]);
% minVal = rad2deg*min(output.euler_angles_ic(:,3))-margin;
% maxVal = rad2deg*max(output.euler_angles_ic(:,3))+margin;
% ylim([minVal maxVal]);
grid on;
ylabel('Yaw ic (deg)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

fileName='euler_angle_ic_estimates.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

%% plot position from vison to world frame estimates
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', 'Position vw');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

subplot(3,1,1);
plot(output.time_lapsed,[output.position_vw(:,1),output.position_vw(:,1)+2*sqrt(output.state_variances(:,33)),output.position_vw(:,1)-2*sqrt(output.state_variances(:,33))]);
grid on;
titleText=strcat({'Position from Vison to World Frame Estimates'},runIdentifier);
title(titleText);
ylabel('X vw (m)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

subplot(3,1,2);
plot(output.time_lapsed,[output.position_vw(:,2),output.position_vw(:,2)+2*sqrt(output.state_variances(:,34)),output.position_vw(:,2)-2*sqrt(output.state_variances(:,34))]);
grid on;
ylabel('Y vw (m)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

subplot(3,1,3);
plot(output.time_lapsed,[output.position_vw(:,3),output.position_vw(:,3)+2*sqrt(output.state_variances(:,35)),output.position_vw(:,3)-2*sqrt(output.state_variances(:,35))]);
grid on;
ylabel('Z vw (m)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

fileName='position_vw_estimates.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

%% plot 3d position from vison to world frame estimates
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', '3D Position vw');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

plot3(output.position_vw(:,1),output.position_vw(:,2),output.position_vw(:,3));
grid on;
titleText=strcat({'3D NED Position from Vision to World Frame Estimates'},runIdentifier);
title(titleText);
zlabel('Z vw (m)');
ylabel('Y vw (m)');
xlabel('X vw (m)');

fileName='3d_position_vw_estimates.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

%% plot euler angle from vision to world frame estimates
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', 'Euler Angle vw');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

margin = 1;

subplot(3,1,1);
plot(output.time_lapsed,[output.euler_angles_vw(:,1)*rad2deg,output.euler_angles_vw(:,1)*rad2deg-2*sqrt(output.euler_variances_vw(:,1)*rad2deg),output.euler_angles_vw(:,1)*rad2deg+2*sqrt(output.euler_variances_vw(:,1)*rad2deg)]);
minVal = rad2deg*min(output.euler_angles_vw(:,1))-margin;
maxVal = rad2deg*max(output.euler_angles_vw(:,1))+margin;
ylim([minVal maxVal]);
grid on;
titleText=strcat({'Euler Angle from Vision to World Frame Estimates'},runIdentifier);
title(titleText);
ylabel('Roll vw (deg)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

subplot(3,1,2);
plot(output.time_lapsed,[output.euler_angles_vw(:,2)*rad2deg,output.euler_angles_vw(:,2)*rad2deg-2*sqrt(output.euler_variances_vw(:,2)*rad2deg),output.euler_angles_vw(:,2)*rad2deg+2*sqrt(output.euler_variances_vw(:,2)*rad2deg)]);
minVal = rad2deg*min(output.euler_angles_vw(:,2))-margin;
maxVal = rad2deg*max(output.euler_angles_vw(:,2))+margin;
ylim([minVal maxVal]);
grid on;
ylabel('Pitch vw (deg)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

subplot(3,1,3);
plot(output.time_lapsed,[output.euler_angles_vw(:,3)*rad2deg,output.euler_angles_vw(:,3)*rad2deg-2*sqrt(output.euler_variances_vw(:,3)*rad2deg),output.euler_angles_vw(:,3)*rad2deg+2*sqrt(output.euler_variances_vw(:,3)*rad2deg)]);
minVal = rad2deg*min(output.euler_angles_vw(:,3))-margin;
maxVal = rad2deg*max(output.euler_angles_vw(:,3))+margin;
ylim([minVal maxVal]);
grid on;
ylabel('Yaw vw (deg)');
xlabel('time (sec)');
legend('estimate','upper 95% bound','lower 95% bound');

fileName='euler_angle_vw_estimates.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

% Measurements
%% plot position from camera to vision frame measurements
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', 'Position cv');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

subplot(3,1,1);
plot(output.time_lapsed,output.position_cv(:,1));
grid on;
titleText=strcat({'Position from Camera to Vision Frame Measurements'},runIdentifier);
title(titleText);
ylabel('X cv (m)');
xlabel('time (sec)');

subplot(3,1,2);
plot(output.time_lapsed,output.position_cv(:,2));
grid on;
ylabel('Y cv (m)');
xlabel('time (sec)');

subplot(3,1,3);
plot(output.time_lapsed,output.position_cv(:,3));
grid on;
ylabel('Z cv (m)');
xlabel('time (sec)');

fileName='position_cv_measurements.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

%% plot 3d position from camera to vision frame measurements
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', '3D Position cv');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

plot3(output.position_cv(:,1),output.position_cv(:,2),output.position_cv(:,3));
grid on;
titleText=strcat({'3D Position from Vision to World Frame Measurements'},runIdentifier);
title(titleText);
zlabel('Z cv (m)');
ylabel('Y cv (m)');
xlabel('X cv (m)');

fileName='3d_position_cv_measurements.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

%% plot euler angle from vision to camera frame measurements
figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', 'Euler Angle vc');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

margin = 1;

subplot(3,1,1);
plot(output.time_lapsed,output.euler_angles_vc(:,1)*rad2deg);
minVal = rad2deg*min(output.euler_angles_vc(:,1))-margin;
maxVal = rad2deg*max(output.euler_angles_vc(:,1))+margin;
ylim([minVal maxVal]);
grid on;
titleText=strcat({'Euler Angle from Vision to World Frame Measurements'},runIdentifier);
title(titleText);
ylabel('Roll vc (deg)');
xlabel('time (sec)');

subplot(3,1,2);
plot(output.time_lapsed,output.euler_angles_vc(:,2)*rad2deg);
minVal = rad2deg*min(output.euler_angles_vc(:,2))-margin;
maxVal = rad2deg*max(output.euler_angles_vc(:,2))+margin;
ylim([minVal maxVal]);
grid on;
ylabel('Pitch vc (deg)');
xlabel('time (sec)');

subplot(3,1,3);
plot(output.time_lapsed,output.euler_angles_vc(:,3)*rad2deg);
minVal = rad2deg*min(output.euler_angles_vc(:,3))-margin;
maxVal = rad2deg*max(output.euler_angles_vc(:,3))+margin;
ylim([minVal maxVal]);
grid on;
ylabel('Yaw vc (deg)');
xlabel('time (sec)');

fileName='euler_angle_vc_measurements.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

% Debug

figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', 'Compare iw and cv 3D Position');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

plot3(output.position_NED_iw(:,1),output.position_NED_iw(:,2),output.position_NED_iw(:,3),'b');
grid on;
hold on;
plot3(output.position_cv(:,1),output.position_cv(:,2),output.position_cv(:,3),'r');
titleText=strcat({'Compare iw and cv 3D Position'},runIdentifier);
title(titleText);
zlabel('Z (m)');
ylabel('Y (m)');
xlabel('X (m)');
legend('iw 3D Position','cv 3D Position');

fileName='compare_iw_cv_3d_position.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);

figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait','NumberTitle', 'off', 'Name', 'Compare wi and vc Attitude');
h=gcf;
set(h,'PaperOrientation','portrait');
set(h,'PaperUnits','normalized');
set(h,'PaperPosition', [0 0 1 1]);

subplot(3,1,1);
plot(output.euler_angles_wi(:,1),'b');
grid on;
hold on;
plot(output.euler_angles_vc(:,1),'r');
titleText=strcat({'Compare wi and vc Attitude'},runIdentifier);
title(titleText);
ylabel('Roll (deg)');
xlabel('time (sec)');
legend('iw Roll','cv Roll');

subplot(3,1,2);
plot(output.euler_angles_wi(:,2),'b');
grid on;
hold on;
plot(output.euler_angles_vc(:,2),'r');
ylabel('Pitch (deg)');
xlabel('time (sec)');
legend('iw Pitch','cv Pitch');

subplot(3,1,3);
plot(output.euler_angles_wi(:,3),'b');
grid on;
hold on;
plot(output.euler_angles_vc(:,3),'r');
ylabel('Yaw (deg)');
xlabel('time (sec)');
legend('iw Yaw','cv Yaw');

fileName='compare_iw_cv_attitude.png';
fullFileName = fullfile(folder, fileName);
saveas(h,fullFileName);



%% plot velocity innovations
if isfield(output.innovations,'vel_innov')
    
    figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
    h=gcf;
    set(h,'PaperOrientation','portrait');
    set(h,'PaperUnits','normalized');
    set(h,'PaperPosition', [0 0 1 1]);
    
    subplot(3,1,1);
    plot(output.innovations.vel_time_lapsed',[output.innovations.vel_innov(:,1),sqrt(output.innovations.vel_innov_var(:,1)),-sqrt(output.innovations.vel_innov_var(:,1))]);
    grid on;
    titleText=strcat({'Velocity Innovations and Variances'},runIdentifier);
    title(titleText);
    ylabel('North (m/s)');
    xlabel('time (sec)');
    legend('innovation','variance sqrt','variance sqrt');
    
    subplot(3,1,2);
    plot(output.innovations.vel_time_lapsed',[output.innovations.vel_innov(:,2),sqrt(output.innovations.vel_innov_var(:,2)),-sqrt(output.innovations.vel_innov_var(:,2))]);
    grid on;
    ylabel('East (m/s)');
    xlabel('time (sec)');
    legend('innovation','variance sqrt','variance sqrt');
    
    subplot(3,1,3);
    plot(output.innovations.vel_time_lapsed',[output.innovations.vel_innov(:,3),sqrt(output.innovations.vel_innov_var(:,3)),-sqrt(output.innovations.vel_innov_var(:,3))]);
    grid on;
    ylabel('Down (m/s)');
    xlabel('time (sec)');
    legend('innovation','variance sqrt','variance sqrt');
    
    fileName='velocity_fusion.png';
    fullFileName = fullfile(folder, fileName);
    saveas(h,fullFileName);
end

%% plot position innovations
if isfield(output.innovations,'posInnov')
    figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
    h=gcf;
    set(h,'PaperOrientation','portrait');
    set(h,'PaperUnits','normalized');
    set(h,'PaperPosition', [0 0 1 1]);
    
    subplot(3,1,1);
    plot(output.innovations.vel_time_lapsed',[output.innovations.posInnov(:,1),sqrt(output.innovations.posInnovVar(:,1)),-sqrt(output.innovations.posInnovVar(:,1))]);
    grid on;
    titleText=strcat({'Position Innovations and Variances'},runIdentifier);
    title(titleText);
    ylabel('North (m)');
    xlabel('time (sec)');
    legend('innovation','variance sqrt','variance sqrt');
    
    subplot(3,1,2);
    plot(output.innovations.vel_time_lapsed',[output.innovations.posInnov(:,2),sqrt(output.innovations.posInnovVar(:,2)),-sqrt(output.innovations.posInnovVar(:,2))]);
    grid on;
    ylabel('East (m)');
    xlabel('time (sec)');
    legend('innovation','variance sqrt','variance sqrt');
    
    subplot(3,1,3);
    plot(output.innovations.hgt_time_lapsed',[output.innovations.hgtInnov(:),sqrt(output.innovations.hgtInnovVar(:)),-sqrt(output.innovations.hgtInnovVar(:))]);
    grid on;
    ylabel('Up (m)');
    xlabel('time (sec)');
    legend('innovation','variance sqrt','variance sqrt');
    
    fileName='position_fusion.png';
    fullFileName = fullfile(folder, fileName);
    saveas(h,fullFileName);
end

%% plot magnetometer innovations
if isfield(output.innovations,'magInnov')
    
    figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
    h=gcf;
    set(h,'PaperOrientation','portrait');
    set(h,'PaperUnits','normalized');
    set(h,'PaperPosition', [0 0 1 1]);
    subplot(4,1,1);
    plot(output.innovations.mag_time_lapsed,[output.innovations.magInnov(:,1)';sqrt(output.innovations.magInnovVar(:,1))';-sqrt(output.innovations.magInnovVar(:,1))']);
    ylim([-0.15 0.15]);
    grid on;
    title(strcat({'Magnetometer Innovations and Variances'},runIdentifier));
    ylabel('X (gauss)');
    xlabel('time (sec)');
    legend('innovation','innovation variance sqrt','innovation variance sqrt');
    subplot(4,1,2);
    plot(output.innovations.mag_time_lapsed,[output.innovations.magInnov(:,2)';sqrt(output.innovations.magInnovVar(:,2))';-sqrt(output.innovations.magInnovVar(:,2))']);
    ylim([-0.15 0.15]);
    grid on;
    ylabel('Y (gauss)');
    xlabel('time (sec)');
    legend('innovation','innovation variance sqrt','innovation variance sqrt');
    subplot(4,1,3);
    plot(output.innovations.mag_time_lapsed,[output.innovations.magInnov(:,3)';sqrt(output.innovations.magInnovVar(:,3))';-sqrt(output.innovations.magInnovVar(:,3))']);
    ylim([-0.15 0.15]);
    grid on;
    ylabel('Z (gauss)');
    xlabel('time (sec)');
    legend('innovation','innovation variance sqrt','innovation variance sqrt');
    subplot(4,1,4);
    plot(output.innovations.mag_time_lapsed,output.innovations.magLength);
    ylim([0 0.7]);
    grid on;
    title(strcat({'Magnetic Flux'},runIdentifier));
    ylabel('Flux (Gauss)');
    xlabel('time (sec)');
    fileName='magnetometer_fusion.png';
    fullFileName = fullfile(folder, fileName);
    saveas(h,fullFileName);
    
end

%% plot magnetic yaw innovations
if isfield(output.innovations,'hdgInnov')
    
    figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
    h=gcf;
    set(h,'PaperOrientation','portrait');
    set(h,'PaperUnits','normalized');
    set(h,'PaperPosition', [0 0 1 1]);
    
    subplot(2,1,1);
    plot(output.innovations.mag_time_lapsed,[output.innovations.hdgInnov*rad2deg;sqrt(output.innovations.hdgInnovVar)*rad2deg;-sqrt(output.innovations.hdgInnovVar)*rad2deg]);
    ylim([-30 30]);
    grid on;
    title(strcat({'Magnetic Heading Innovations and Variances'},runIdentifier));
    ylabel('yaw innovation (deg)');
    xlabel('time (sec)');
    legend('innovation','innovation variance sqrt','innovation variance sqrt');
    subplot(2,1,2);
    plot(output.innovations.mag_time_lapsed,output.innovations.magLength);
    ylim([0 0.7]);
    grid on;
    title(strcat({'Magnetic Flux'},runIdentifier));
    ylabel('Flux (Gauss)');
    xlabel('time (sec)');
    fileName='magnetometer_fusion.png';
    fullFileName = fullfile(folder, fileName);
    saveas(h,fullFileName);
    
end

%% plot optical flow innovations
if isfield(output.innovations,'flowInnov')
    
    figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
    h=gcf;
    set(h,'PaperOrientation','portrait');
    set(h,'PaperUnits','normalized');
    set(h,'PaperPosition', [0 0 1 1]);
    subplot(2,1,1);
    plot(output.innovations.flow_time_lapsed,[output.innovations.flowInnov(:,1)';sqrt(output.innovations.flowInnovVar(:,1))';-sqrt(output.innovations.flowInnovVar(:,1))']);
    ylim([-1.0 1.0]);
    grid on;
    title(strcat({'Optical Flow Innovations and Variances'},runIdentifier));
    ylabel('X (rad/sec)');
    xlabel('time (sec)');
    legend('innovation','innovation variance sqrt','innovation variance sqrt');
    subplot(2,1,2);
    plot(output.innovations.flow_time_lapsed,[output.innovations.flowInnov(:,2)';sqrt(output.innovations.flowInnovVar(:,2))';-sqrt(output.innovations.flowInnovVar(:,2))']);
    ylim([-1.0 1.0]);
    grid on;
    ylabel('Y (rad/sec)');
    xlabel('time (sec)');
    legend('innovation','innovation variance sqrt','innovation variance sqrt');
    fileName='optical_flow_fusion.png';
    fullFileName = fullfile(folder, fileName);
    saveas(h,fullFileName);
    
end
%% plot ZED camera innovations
if isfield(output.innovations,'bodyVelInnov')
    
    figure('Units','Pixels','Position',plotDimensions,'PaperOrientation','portrait');
    h=gcf;
    set(h,'PaperOrientation','portrait');
    set(h,'PaperUnits','normalized');
    set(h,'PaperPosition', [0 0 1 1]);
    
    subplot(3,1,1);
    plot(output.innovations.bodyVel_time_lapsed,[output.innovations.bodyVelInnov(:,1)';sqrt(output.innovations.bodyVelInnovVar(:,1))';-sqrt(output.innovations.bodyVelInnovVar(:,1))']);
    grid on;
    title(strcat({'ZED Camera Innovations and Variances'},runIdentifier));
    ylabel('X (m/sec)');
    xlabel('time (sec)');
    legend('innovation','innovation variance sqrt','innovation variance sqrt');
    
    subplot(3,1,2);
    plot(output.innovations.bodyVel_time_lapsed,[output.innovations.bodyVelInnov(:,2)';sqrt(output.innovations.bodyVelInnovVar(:,2))';-sqrt(output.innovations.bodyVelInnovVar(:,2))']);
    grid on;
    ylabel('Y (m/sec)');
    xlabel('time (sec)');
    legend('innovation','innovation variance sqrt','innovation variance sqrt');
    
    subplot(3,1,3);
    plot(output.innovations.bodyVel_time_lapsed,[output.innovations.bodyVelInnov(:,3)';sqrt(output.innovations.bodyVelInnovVar(:,3))';-sqrt(output.innovations.bodyVelInnovVar(:,3))']);
    grid on;
    ylabel('Z (m/sec)');
    xlabel('time (sec)');
    legend('innovation','innovation variance sqrt','innovation variance sqrt');
    
    fileName='zed_camera_fusion.png';
    fullFileName = fullfile(folder, fileName);
    saveas(h,fullFileName);

end
