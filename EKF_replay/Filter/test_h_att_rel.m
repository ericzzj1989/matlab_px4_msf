% figure;
% plot3(output.position_NED_iw(:,1),output.position_NED_iw(:,2),output.position_NED_iw(:,3),'b');
% grid on;
% hold on;
% plot3(output.position_cv(:,1),output.position_cv(:,2),output.position_cv(:,3),'r');
% 
% figure;
% plot(output.euler_angles_wi(:,1),'b');%,output.euler_angles_wi(:,2),output.euler_angles_wi(:,3),'b');
% grid on;
% hold on;
% plot(output.euler_angles_vc(:,1),'r');%,output.euler_angles_vc(:,2),output.euler_angles_vc(:,3),'r');

addpath('../Common');

syms q0_wi_old q1_wi_old q2_wi_old q3_wi_old real;
syms q0_ic_old q1_ic_old q2_ic_old q3_ic_old real;
syms q0_vw_old q1_vw_old q2_vw_old q3_vw_old real;

syms q0_wi_new q1_wi_new q2_wi_new q3_wi_new real;
syms q0_ic_new q1_ic_new q2_ic_new q3_ic_new real;
syms q0_vw_new q1_vw_new q2_vw_new q3_vw_new real;

% qvw_Left = q0_vw * eye(4) + [0 -q1_vw -q2_vw -q3_vw;
%                              q1_vw 0 -q3_vw q2_vw;
%                              q2_vw q3_vw 0 -q1_vw;
%                              q3_vw -q2_vw q1_vw 0];
    
Mult_qvw_qwi_old = QuatMult([q0_vw_old;q1_vw_old;q2_vw_old;q3_vw_old],[q0_wi_old;q1_wi_old;q2_wi_old;q3_wi_old]); % qvw_Left * [q0_wi q1_wi q2_wi q3_wi]';

% Mult_qvw_qwi_Left = Mult_qvw_qwi(1) * eye(4) + [0 -Mult_qvw_qwi(2) -Mult_qvw_qwi(3) -Mult_qvw_qwi(4);
%                                                 Mult_qvw_qwi(2) 0 -Mult_qvw_qwi(4) Mult_qvw_qwi(3);
%                                                 Mult_qvw_qwi(3) Mult_qvw_qwi(4) 0 -Mult_qvw_qwi(2);
%                                                 Mult_qvw_qwi(4) -Mult_qvw_qwi(3) Mult_qvw_qwi(2) 0];

Quat_vc_old = QuatMult(Mult_qvw_qwi_old,[q0_ic_old;q1_ic_old;q2_ic_old;q3_ic_old]); % Mult_qvw_qwi_Left * [q0_ic q1_ic q2_ic q3_ic]';


Mult_qvw_qwi_new = QuatMult([q0_vw_new;q1_vw_new;q2_vw_new;q3_vw_new],[q0_wi_new;q1_wi_new;q2_wi_new;q3_wi_new]);

Quat_vc_new = QuatMult(Mult_qvw_qwi_new,[q0_ic_new;q1_ic_new;q2_ic_new;q3_ic_new]);

Quat_vc_new_conj = [Quat_vc_new(1);-Quat_vc_new(2);-Quat_vc_new(3);-Quat_vc_new(4)];

z_q_relative = QuatMult(Quat_vc_new_conj,Quat_vc_old);

H_z_q_relative_qwi_old = jacobian(z_q_relative,[q0_wi_old;q1_wi_old;q2_wi_old;q3_wi_old]);

H_z_q_relative_qic_old = jacobian(z_q_relative,[q0_ic_old;q1_ic_old;q2_ic_old;q3_ic_old]);

H_z_q_relative_qvw_old = jacobian(z_q_relative,[q0_vw_old;q1_vw_old;q2_vw_old;q3_vw_old]);


H_z_q_relative_qwi_new = jacobian(z_q_relative,[q0_wi_new;q1_wi_new;q2_wi_new;q3_wi_new]);

H_z_q_relative_qic_new = jacobian(z_q_relative,[q0_ic_new;q1_ic_new;q2_ic_new;q3_ic_new]);

H_z_q_relative_qvw_new = jacobian(z_q_relative,[q0_vw_new;q1_vw_new;q2_vw_new;q3_vw_new]);