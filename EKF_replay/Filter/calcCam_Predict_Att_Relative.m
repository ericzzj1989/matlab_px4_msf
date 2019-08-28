function Cam_Predict_Att_Relative = calcCam_Predict_Att_Relative(states_old,states_new)

q0_wi_old = states_old(1);
q1_wi_old = states_old(2);
q2_wi_old = states_old(3);
q3_wi_old = states_old(4);

q0_ic_old = states_old(29);
q1_ic_old = states_old(30);
q2_ic_old = states_old(31);
q3_ic_old = states_old(32);

q0_vw_old = states_old(36);
q1_vw_old = states_old(37);
q2_vw_old = states_old(38);
q3_vw_old = states_old(39);


q0_wi_new = states_new(1);
q1_wi_new = states_new(2);
q2_wi_new = states_new(3);
q3_wi_new = states_new(4);

q0_ic_new = states_new(29);
q1_ic_new = states_new(30);
q2_ic_new = states_new(31);
q3_ic_new = states_new(32);

q0_vw_new = states_new(36);
q1_vw_new = states_new(37);
q2_vw_new = states_new(38);
q3_vw_new = states_new(39);


Mult_qvw_qwi_old = QuatMult([q0_vw_old;q1_vw_old;q2_vw_old;q3_vw_old],[q0_wi_old;q1_wi_old;q2_wi_old;q3_wi_old]);

Quat_vc_old = QuatMult(Mult_qvw_qwi_old,[q0_ic_old;q1_ic_old;q2_ic_old;q3_ic_old]);


Mult_qvw_qwi_new = QuatMult([q0_vw_new;q1_vw_new;q2_vw_new;q3_vw_new],[q0_wi_new;q1_wi_new;q2_wi_new;q3_wi_new]);

Quat_vc_new = QuatMult(Mult_qvw_qwi_new,[q0_ic_new;q1_ic_new;q2_ic_new;q3_ic_new]);

Quat_vc_new_conj = [Quat_vc_new(1);-Quat_vc_new(2);-Quat_vc_new(3);-Quat_vc_new(4)];


Cam_Predict_Att_Relative = QuatMult(Quat_vc_new_conj,Quat_vc_old);