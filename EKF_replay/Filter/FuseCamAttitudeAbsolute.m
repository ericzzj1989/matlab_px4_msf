function [...
    states, ... % state vector after fusion of measurements
    P, ... % state covariance matrix after fusion of corrections
    innovation_att, ... % camera attitude innovations 
    innovation_pos, ... % camera position innovations
    varInnov] ... % camera pose innovation variance
    = FuseCamAttitudeAbsolute( ...
    param, ... % parameter
    states, ... % predicted states from the INS
    P, ... % predicted covariance
    measAtt, ... % camera to vision attitude measurements
    measPos, ... % camera to vision position measurements
    gateSize, ... % Size of the innovation consistency check gate (std-dev)
    R_CAM_ATT, ... % camera attitude absolute observation variance
    R_CAM_POS) % camera position absolute observation variance


% Calculate the observation Jacobian
H_ATT = calcH_Att_Absolute(param,states);
H_POS = calcH_Pos_Absolute(param,states);
H = [H_ATT; H_POS];

% Calculate the predicts 
quat_vc_predict = calcCam_Predict_Att_Absolute(states);
pos_cv_predict = calcCam_Predict_Pos_Absolute(states);

% Calculate measurement noise
R = diag([R_CAM_ATT*[1;1;1;1]; R_CAM_POS*[1;1;1]]);


%% Calculate Kalman gains and update states and covariances (EKF process)

% calculate the innovation variance
varInnov = H * P * transpose(H) + R;

% Calculate the Kalman gains
K = (P * transpose(H)) / varInnov;

% predict_conj = [quat_vc_predict(1);-quat_vc_predict(2);-quat_vc_predict(3);-quat_vc_predict(4)];
% 
% innovation = QuatMult(predict_conj, measAtt);

% Calculate the innovation
positive_val = abs(quat_vc_predict(1) - measAtt(1));
negative_val = abs(quat_vc_predict(1) + measAtt(1) );

if positive_val < negative_val
    innovation_att =  [quat_vc_predict(1);quat_vc_predict(2);quat_vc_predict(3);quat_vc_predict(4)] - measAtt;
else    
    innovation_att =  [quat_vc_predict(1);quat_vc_predict(2);quat_vc_predict(3);quat_vc_predict(4)] + measAtt;
end

innovation_pos = pos_cv_predict - measPos;

innovation = [innovation_att; innovation_pos];

% if abs(innovation(1)) > 0.2 || abs(innovation(2)) > 0.2 || abs(innovation(3)) > 0.2 || abs(innovation(4)) > 0.2
%     innovation = [0;0;0;0];
%     return;
% end

% Calculate state corrections
correction = K * innovation;

% Apply the state corrections
states = states - correction;

% normalise the updated quaternion states
quat_wiNorm = sqrt(states(1)^2 + states(2)^2 + states(3)^2 + states(4)^2);
if (quat_wiNorm > 1e-12)
    states(1:4) = states(1:4) / quat_wiNorm;
end

quat_icNorm = sqrt(states(29)^2 + states(30)^2 + states(31)^2 + states(32)^2);
if (quat_icNorm > 1e-12)
    states(29:32) = states(29:32) / quat_icNorm;
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