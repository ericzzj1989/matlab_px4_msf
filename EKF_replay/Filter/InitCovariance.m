function covariance = InitCovariance(param,dt,gps_alignment,gps_data)

% Define quaternion state errors
Sigma_quat = param.alignment.quatErr * [1;1;1;1];

% % Define velocity state errors
% if (gps_alignment == 1)
%     Sigma_velocity = gps_data.spd_error(gps_data.start_index) * [1;1;1];
% else
%     Sigma_velocity = [param.alignment.velErrNE;param.alignment.velErrNE;param.alignment.velErrD];
% end
% 
% % Define position state errors
% if (gps_alignment == 1)
%     Sigma_position = gps_data.pos_error(gps_data.start_index) * [1;1;0] + [0;0;param.alignment.hgtErr];
% else
%     Sigma_position = [param.alignment.posErrNE;param.alignment.posErrNE;param.alignment.hgtErr];
% end

% Define delta angle bias state errors
Sigma_dAngBias = param.alignment.delAngBiasErr*dt*[1;1;1];

% Define delta velocity bias state errors
Sigma_dVelBias = param.alignment.delVelBiasErr*dt*[1;1;1];

% Define magnetic field state errors
Sigma_magNED = [param.alignment.magErrNED;param.alignment.magErrNED;param.alignment.magErrNED]; % 1 Sigma uncertainty in initial NED mag field
Sigma_magXYZ = [param.alignment.magErrXYZ;param.alignment.magErrXYZ;param.alignment.magErrXYZ]; % 1 Sigma uncertainty in initial XYZ mag sensor offset

% Define wind velocity state errors
Sigma_wind = param.alignment.windErrNE * [1;1];

% TO DO: Define camera pose state errors
Sigma_lamda = param.alignment.lamdaErr;
Sigma_posci = param.alignment.posErrci * [1;1;1];
Sigma_quatic = param.alignment.quatErric * [1;1;1;1];
Sigma_posvw = param.alignment.posErrvw * [1;1;1];
Sigma_quatvw = param.alignment.quatErrvw * [1;1;1;1];

% Convert to variances and write to covariance matrix diagonals
covariance = diag([Sigma_quat;[0;0;0];[0;0;0];Sigma_dAngBias;Sigma_dVelBias;Sigma_magNED;Sigma_magXYZ;Sigma_wind;Sigma_lamda;Sigma_posci;Sigma_quatic;Sigma_posvw;Sigma_quatvw].^2);
end