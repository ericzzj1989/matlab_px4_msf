function [...
    states, ... % state vector after fusion of measurements
    P, ... % state covariance matrix after fusion of corrections
    innovation,... % camera pose innovations (m)
    varInnov] ... % camera pose innovation variance (m^2)
    = FuseCamPositionAbsolute( ...
    param, ... % parameter
    states, ... % predicted states from the INS
    P, ... % predicted covariance
    measPos, ... % camera to vision position measurements
    gateSize, ... % Size of the innovation consistency check gate (std-dev)
    R_CAM) % camera position absolute observation variance (m)^2   

% lamda = states(25);
% 
% q0_wi = states(1);
% q1_wi = states(2);
% q2_wi = states(3);
% q3_wi = states(4);
% 
% q0_vw = states(36);
% q1_vw = states(37);
% q2_vw = states(38);
% q3_vw = states(39);
% 
% px_iw = states(8);
% py_iw = states(9);
% pz_iw = states(10);
% 
% px_ci = states(26);
% py_ci = states(27);
% pz_ci = states(28);
% 
% px_vw = states(33);
% py_vw = states(34);
% pz_vw = states(35);

% innovation = zeros(3,1);
% varInnov = zeros(3,3);
% H = zeros(3,39);

% Calculate the observation Jacobian
H = calcH_Pos_Absolute(param,states);
p_cv_predict = calcCam_Predict_Pos_Absolute(states);

% pos_meas_diff = p_cv - H * states;

% Calculate Kalman gains and update states and covariances (EKF process)

% calculate the  innovation variance
varInnov = H * P * transpose(H) + R_CAM * eye(3);

% Calculate the Kalman gains
K = (P * transpose(H)) / varInnov;

% Calculate the innovation
innovation = p_cv_predict - measPos; %  (H * states + R_CAM) - measPos;

% Apply an innovation consistency check
% for obsIndex = 1:3
%     
%     if (innovation(obsIndex)^2 / (gateSize^2 * varInnov(obsIndex))) > 1.0
%         return;
%     end
%     
% end

% Calculate state corrections
xk = K * innovation;

% Apply the state corrections
states = states - xk;

% normalise the updated quaternion states
quat_wiNorm = sqrt(states(1)^2 + states(2)^2 + states(3)^2 + states(4)^2);
if (quat_wiNorm > 1e-12)
    states(1:4) = states(1:4) / quat_wiNorm;
end

quat_vwNorm = sqrt(states(36)^2 + states(37)^2 + states(38)^2 + states(39)^2);
if (quat_vwNorm > 1e-12)
    states(36:39) = states(36:39) / quat_vwNorm;
end

% Update the covariance
P = P - K * H * P; 

% Force symmetry on the covariance matrix to prevent ill-conditioning
P = 0.5*(P + transpose(P));

% ensure diagonals are positive
for i=1:39
    if P(i,i) < 0
        P(i,i) = 0;
    end
end

end