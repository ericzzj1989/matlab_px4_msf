function Cam_Predict_Att_Absolute = calcCam_Predict_Att_Absolute(states)

q0_wi = states(1);
q1_wi = states(2);
q2_wi = states(3);
q3_wi = states(4);

q0_ic = states(29);
q1_ic = states(30);
q2_ic = states(31);
q3_ic = states(32);

q0_vw = states(36);
q1_vw = states(37);
q2_vw = states(38);
q3_vw = states(39);

% qvw_Left = q0_vw * eye(4) + [0 -q1_vw -q2_vw -q3_vw;
%                              q1_vw 0 -q3_vw q2_vw;
%                              q2_vw q3_vw 0 -q1_vw;
%                              q3_vw -q2_vw q1_vw 0];
    
Mult_qvw_qwi = QuatMult([q0_vw;q1_vw;q2_vw;q3_vw],[q0_wi;q1_wi;q2_wi;q3_wi]); % qvw_Left * [q0_wi q1_wi q2_wi q3_wi]';

% Mult_qvw_qwi_Left = Mult_qvw_qwi(1) * eye(4) + [0 -Mult_qvw_qwi(2) -Mult_qvw_qwi(3) -Mult_qvw_qwi(4);
%                                                 Mult_qvw_qwi(2) 0 -Mult_qvw_qwi(4) Mult_qvw_qwi(3);
%                                                 Mult_qvw_qwi(3) Mult_qvw_qwi(4) 0 -Mult_qvw_qwi(2);
%                                                 Mult_qvw_qwi(4) -Mult_qvw_qwi(3) Mult_qvw_qwi(2) 0];

Cam_Predict_Att_Absolute = QuatMult(Mult_qvw_qwi,[q0_ic;q1_ic;q2_ic;q3_ic]); % Mult_qvw_qwi_Left * [q0_ic q1_ic q2_ic q3_ic]';