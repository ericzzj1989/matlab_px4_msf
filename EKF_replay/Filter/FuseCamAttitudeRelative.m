function [...
    states, ... % state vector after fusion of measurements
    P, ... % state covariance matrix after fusion of corrections
    innovation_att, ... % camera attitude innovations
    innovation_pos, ... % camera position innovations
    varInnov] ... % camera pose innovation variance
    = FuseCamAttitudeRelative( ...
    param, ... % parameter
    states_old, ... % previous state vector
    states_new, ... % predicted states from the INS
    P_old, ...% previous state covariance matrix
    P_new, ... % predicted covariance
    F_old, ... %
    F_new, ... %
    meas_att_old, ... % previous camera to vision attitude measurements
    meas_att_new, ... % current camera to vision attitude measurements
    meas_pos_old, ...
    meas_pos_new, ...
    gateSize, ... % Size of the innovation consistency check gate (std-dev)
    R_CAM_ATT, ... % camera attitude relative observation variance (m)^2
    R_CAM_POS) % camera position relative observation variance (m)^2 


P_SC = zeros(78,78);

% Get the accumulated system dynamics
F_accum = F_new;

%      | P_kk       P_kk * F' |
%  P = |                      |
%      | F * P_kk   P_mk      |
P_SC(1:39, 1:39) = P_old;
P_SC(1:39, 40:78) = P_old * transpose(F_accum);
P_SC(40:78, 1:39) = F_accum * P_old;
P_SC(40:78, 40:78) = P_new;

% Calculate the observation Jacobian
H_SC_ATT = calcH_Att_Relative(param,states_old,states_new);
H_SC_POS = calcH_Pos_Relative(param,states_old,states_new);
H_SC = [H_SC_ATT; H_SC_POS];
H_old = H_SC(1:7,1:39);
H_new = H_SC(1:7,40:78);

% Calculate the predicts and measurements
Predict_Att_Relative = calcCam_Predict_Att_Relative(states_old,states_new);
meas_att_new_conj = [meas_att_new(1);-meas_att_new(2);-meas_att_new(3);-meas_att_new(4)];
Measurements_Att_Relative = QuatMult(meas_att_new_conj,meas_att_old);

Predict_Pos_Relative = calcCam_Predict_Pos_Relative(states_old,states_new);
Measurements_Pos_Relative = meas_pos_new - meas_pos_old;

% Calculate measurement noise
R = diag([R_CAM_ATT*[1;1;1;1]; R_CAM_POS*[1;1;1]]);


%% Calculate Kalman gains and update states and covariances (EKF process)

% Calculate the innovation variance
varInnov = H_SC * P_SC * transpose(H_SC) + R;

% Calculate the Kalman gains in in relative measurement
K = (F_accum * P_old * transpose(H_old) + P_new * transpose(H_new)) / varInnov;

% Predict_conj = [Predict_Att_Relative(1);-Predict_Att_Relative(2);-Predict_Att_Relative(3);-Predict_Att_Relative(4)];
% 
% innovation = QuatMult(Predict_conj, Measurements_Att_Relative);

% Calculate the innovation
positive_val = abs(Predict_Att_Relative(1) - Measurements_Att_Relative(1));
negative_val = abs(Predict_Att_Relative(1) + Measurements_Att_Relative(1));

if positive_val < negative_val
    innovation_att =  [Predict_Att_Relative(1);Predict_Att_Relative(2);Predict_Att_Relative(3);Predict_Att_Relative(4)] - Measurements_Att_Relative;
else    
    innovation_att =  [Predict_Att_Relative(1);Predict_Att_Relative(2);Predict_Att_Relative(3);Predict_Att_Relative(4)] + Measurements_Att_Relative;
end

innovation_pos = Predict_Pos_Relative - Measurements_Pos_Relative;

innovation = [innovation_att; innovation_pos];

% Calculate state corrections
correction = K * innovation;

% Apply the state corrections
states = states_new - correction;

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
P = P_new - K * varInnov * transpose(K);

% Force symmetry on the covariance matrix to prevent ill-conditioning
P = 0.5 * (P + transpose(P));

% ensure diagonals are positive
for i = 1:39
    if P(i,i) < 0
        P(i,i) = 0;
    end
end

end


