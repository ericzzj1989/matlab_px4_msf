function Cam_Predict_Pos_Relative = calcCam_Predict_Pos_Relative(states_old,states_new)

lamda_old = states_old(25);

q0_wi_old = states_old(1);
q1_wi_old = states_old(2);
q2_wi_old = states_old(3);
q3_wi_old = states_old(4);

q0_vw_old = states_old(36);
q1_vw_old = states_old(37);
q2_vw_old = states_old(38);
q3_vw_old = states_old(39);

px_iw_old = states_old(8);
py_iw_old = states_old(9);
pz_iw_old = states_old(10);

px_ci_old = states_old(26);
py_ci_old = states_old(27);
pz_ci_old = states_old(28);

px_vw_old = states_old(33);
py_vw_old = states_old(34);
pz_vw_old = states_old(35);


lamda_new = states_new(25);

q0_wi_new = states_new(1);
q1_wi_new = states_new(2);
q2_wi_new = states_new(3);
q3_wi_new = states_new(4);

q0_vw_new = states_new(36);
q1_vw_new = states_new(37);
q2_vw_new = states_new(38);
q3_vw_new = states_new(39);

px_iw_new = states_new(8);
py_iw_new = states_new(9);
pz_iw_new = states_new(10);

px_ci_new = states_new(26);
py_ci_new = states_new(27);
pz_ci_new = states_new(28);

px_vw_new = states_new(33);
py_vw_new = states_new(34);
pz_vw_new = states_new(35);


q_wi_old = [q0_wi_old;q1_wi_old;q2_wi_old;q3_wi_old];
q_vw_old = [q0_vw_old;q1_vw_old;q2_vw_old;q3_vw_old];

p_iw_old = [px_iw_old;py_iw_old;pz_iw_old];
p_ci_old = [px_ci_old;py_ci_old;pz_ci_old];
p_vw_old = [px_vw_old;py_vw_old;pz_vw_old];

Pos_cv_old = Quat2Tbn(q_vw_old) * (Quat2Tbn(q_wi_old) * p_ci_old + p_iw_old - p_vw_old) * lamda_old;


q_wi_new = [q0_wi_new;q1_wi_new;q2_wi_new;q3_wi_new];
q_vw_new = [q0_vw_new;q1_vw_new;q2_vw_new;q3_vw_new];

p_iw_new = [px_iw_new;py_iw_new;pz_iw_new];
p_ci_new = [px_ci_new;py_ci_new;pz_ci_new];
p_vw_new = [px_vw_new;py_vw_new;pz_vw_new];

Pos_cv_new = Quat2Tbn(q_vw_new) * (Quat2Tbn(q_wi_new) * p_ci_new + p_iw_new - p_vw_new) * lamda_new;


Cam_Predict_Pos_Relative = Pos_cv_new - Pos_cv_old;