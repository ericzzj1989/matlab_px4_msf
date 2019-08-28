function Cam_Predict_Pos_Absolute = calcCam_Predict_Pos_Absolute(states)

% R_wv(1,1) = q0_vw.^2 + q1_vw.^2 - q2_vw.^2 - q3_vw.^2;
% R_wv(1,2) = 2.*(q1_vw.*q2_vw - q0_vw.*q3_vw);
% R_wv(1,3) = 2.*(q1_vw.*q3_vw + q0_vw.*q2_vw);
% R_wv(2,1) = 2.*(q1_vw.*q2_vw + q0_vw.*q3_vw);
% R_wv(2,2) = q0_vw.^2 - q1_vw.^2 + q2_vw.^2 - q3_vw.^2;
% R_wv(2,3) = 2.*(q2_vw.*q3_vw - q0_vw.*q1_vw);
% R_wv(3,1) = 2.*(q1_vw.*q3_vw -q0_vw.*q2_vw);
% R_wv(3,2) = 2.*(q2_vw.*q3_vw + q0_vw.*q1_vw);
% R_wv(3,3) = q0_vw.^2 - q1_vw.^2 - q2_vw.^2 + q3_vw.^2;
% 
% R_iw(1,1) = q0_wi.^2 + q1_wi.^2 - q2_wi.^2 - q3_wi.^2;
% R_iw(1,2) = 2.*(q1_wi.*q2_wi - q0_wi.*q3_wi);
% R_iw(1,3) = 2.*(q1_wi.*q3_wi + q0_wi.*q2_wi);
% R_iw(2,1) = 2.*(q1_wi.*q2_wi + q0_wi.*q3_wi);
% R_iw(2,2) = q0_wi.^2 - q1_wi.^2 + q2_wi.^2 - q3_wi.^2;
% R_iw(2,3) = 2.*(q2_wi.*q3_wi - q0_wi.*q1_wi);
% R_iw(3,1) = 2.*(q1_wi.*q3_wi -q0_wi.*q2_wi);
% R_iw(3,2) = 2.*(q2_wi.*q3_wi + q0_wi.*q1_wi);
% R_iw(3,3) = q0_wi.^2 - q1_wi.^2 - q2_wi.^2 + q3_wi.^2;
% 
% p_iw = [px_iw py_iw pz_iw]';
% p_ci = [px_ci py_ci pz_ci]';
% p_vw = [px_vw py_vw pz_vw]';
% 
% Pos_cv = R_wv * (R_iw * p_ci + p_iw - p_vw) * lamda;

lamda = states(25);

q0_wi = states(1);
q1_wi = states(2);
q2_wi = states(3);
q3_wi = states(4);

q0_vw = states(36);
q1_vw = states(37);
q2_vw = states(38);
q3_vw = states(39);

px_iw = states(8);
py_iw = states(9);
pz_iw = states(10);

px_ci = states(26);
py_ci = states(27);
pz_ci = states(28);

px_vw = states(33);
py_vw = states(34);
pz_vw = states(35);

q_wi = [q0_wi;q1_wi;q2_wi;q3_wi];
q_vw = [q0_vw;q1_vw;q2_vw;q3_vw];

p_iw = [px_iw;py_iw;pz_iw];
p_ci = [px_ci;py_ci;pz_ci];
p_vw = [px_vw;py_vw;pz_vw];

Cam_Predict_Pos_Absolute = Quat2Tbn(q_vw) * (Quat2Tbn(q_wi) * p_ci + p_iw - p_vw) * lamda;



