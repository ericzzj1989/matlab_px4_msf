addpath('../Common');

% syms q0_wi q1_wi q2_wi q3_wi real;
% % syms q0_ic q1_ic q2_ic q3_ic real;
% syms q0_vw q1_vw q2_vw q3_vw real;
% 
% syms lamda real;
% 
% syms px_iw py_iw pz_iw real;
% syms px_ci py_ci pz_ci real;
% syms px_vw py_vw pz_vw real;


% 
% qvw_Left = q0_vw * eye(4) + [0 -q1_vw -q2_vw -q3_vw;
%                              q1_vw 0 -q3_vw q2_vw;
%                              q2_vw q3_vw 0 -q1_vw;
%                              q3_vw -q2_vw q1_vw 0];
%     
% Mult_qvw_qwi1 = QuatMult([q0_vw;q1_vw;q2_vw;q3_vw],[q0_wi;q1_wi;q2_wi;q3_wi]); 
% 
% Mult_qvw_qwi2 = qvw_Left * [q0_wi q1_wi q2_wi q3_wi]';
% 
% Mult_qvw_qwi_Left = Mult_qvw_qwi1(1) * eye(4) + [0 -Mult_qvw_qwi1(2) -Mult_qvw_qwi1(3) -Mult_qvw_qwi1(4);
%                                                 Mult_qvw_qwi1(2) 0 -Mult_qvw_qwi1(4) Mult_qvw_qwi1(3);
%                                                 Mult_qvw_qwi1(3) Mult_qvw_qwi1(4) 0 -Mult_qvw_qwi1(2);
%                                                 Mult_qvw_qwi1(4) -Mult_qvw_qwi1(3) Mult_qvw_qwi1(2) 0];
% 
% Quat_vc1 = QuatMult(Mult_qvw_qwi2,[q0_ic;q1_ic;q2_ic;q3_ic]); 
% 
% Quat_vc2 = Mult_qvw_qwi_Left * [q0_ic q1_ic q2_ic q3_ic]';
% 
% Diff=Quat_vc1-Quat_vc2;
% [x,y]=find(Diff~=0);

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
% q_wi = [q0_wi;q1_wi;q2_wi;q3_wi];
% q_vw = [q0_vw;q1_vw;q2_vw;q3_vw];
% 
% p_iw = [px_iw;py_iw;pz_iw];
% p_ci = [px_ci;py_ci;pz_ci];
% p_vw = [px_vw;py_vw;pz_vw];
% 
% Pos_cv_1 = R_wv * (R_iw * p_ci + p_iw - p_vw) * lamda;
% 
% Pos_cv_2 = Quat2Tbn(q_vw) * (Quat2Tbn(q_wi) * p_ci + p_iw - p_vw) * lamda;
% 
% Diff=Pos_cv_1-Pos_cv_2;
% [x,y]=find(Diff~=0);



% q0_ic_old = 1;
% q0_vw_old = 1;
% q0_ic_new = 1;
% q0_vw_new = 1;
% q1_ic_old = 0;
% q2_ic_old = 0;
% q3_ic_old = 0;
% q1_vw_old = 0;
% q2_vw_old = 0;
% q3_vw_old = 0;
% q1_ic_new = 0;
% q2_ic_new = 0;
% q3_ic_new = 0;
% q1_vw_new = 0;
% q2_vw_new = 0;
% q3_vw_new = 0;
% 
% syms q0_wi_old q1_wi_old q2_wi_old q3_wi_old real;
% % syms q0_ic_old q1_ic_old q2_ic_old q3_ic_old real;
% % syms q0_vw_old q1_vw_old q2_vw_old q3_vw_old real;
% 
% syms q0_wi_new q1_wi_new q2_wi_new q3_wi_new real;
% % syms q0_ic_new q1_ic_new q2_ic_new q3_ic_new real;
% % syms q0_vw_new q1_vw_new q2_vw_new q3_vw_new real;
% 
% 
% 
% % qvw_Left = q0_vw * eye(4) + [0 -q1_vw -q2_vw -q3_vw;
% %                              q1_vw 0 -q3_vw q2_vw;
% %                              q2_vw q3_vw 0 -q1_vw;
% %                              q3_vw -q2_vw q1_vw 0];
%     
% Mult_qvw_qwi_old = QuatMult([q0_vw_old;q1_vw_old;q2_vw_old;q3_vw_old],[q0_wi_old;q1_wi_old;q2_wi_old;q3_wi_old]); % qvw_Left * [q0_wi q1_wi q2_wi q3_wi]';
% 
% % Mult_qvw_qwi_Left = Mult_qvw_qwi(1) * eye(4) + [0 -Mult_qvw_qwi(2) -Mult_qvw_qwi(3) -Mult_qvw_qwi(4);
% %                                                 Mult_qvw_qwi(2) 0 -Mult_qvw_qwi(4) Mult_qvw_qwi(3);
% %                                                 Mult_qvw_qwi(3) Mult_qvw_qwi(4) 0 -Mult_qvw_qwi(2);
% %                                                 Mult_qvw_qwi(4) -Mult_qvw_qwi(3) Mult_qvw_qwi(2) 0];
% 
% Quat_vc_old = QuatMult(Mult_qvw_qwi_old,[q0_ic_old;q1_ic_old;q2_ic_old;q3_ic_old]); % Mult_qvw_qwi_Left * [q0_ic q1_ic q2_ic q3_ic]';
% 
% 
% Mult_qvw_qwi_new = QuatMult([q0_vw_new;q1_vw_new;q2_vw_new;q3_vw_new],[q0_wi_new;q1_wi_new;q2_wi_new;q3_wi_new]);
% 
% Quat_vc_new = QuatMult(Mult_qvw_qwi_new,[q0_ic_new;q1_ic_new;q2_ic_new;q3_ic_new]);
% 
% Quat_vc_new_conj = [Quat_vc_new(1);-Quat_vc_new(2);-Quat_vc_new(3);-Quat_vc_new(4)];
% 
% 
% z_q_relative = QuatMult(Quat_vc_new_conj,Quat_vc_old);
% 
% H_z_q_relative_qwi_old = jacobian(z_q_relative,[q0_wi_old;q1_wi_old;q2_wi_old;q3_wi_old]);
% % 
% % H_z_q_relative_qic_old = jacobian(z_q_relative,[q0_ic_old;q1_ic_old;q2_ic_old;q3_ic_old]);
% % 
% % H_z_q_relative_qvw_old = jacobian(z_q_relative,[q0_vw_old;q1_vw_old;q2_vw_old;q3_vw_old]);
% % 
% % 
% H_z_q_relative_qwi_new = jacobian(z_q_relative,[q0_wi_new;q1_wi_new;q2_wi_new;q3_wi_new]);
% % 
% % H_z_q_relative_qic_new = jacobian(z_q_relative,[q0_ic_new;q1_ic_new;q2_ic_new;q3_ic_new]);
% % 
% % H_z_q_relative_qvw_new = jacobian(z_q_relative,[q0_vw_new;q1_vw_new;q2_vw_new;q3_vw_new]);


syms q0_wi_old q1_wi_old q2_wi_old q3_wi_old real;
q0_vw_old = 1;
q1_vw_old = 0;
q2_vw_old = 0;
q3_vw_old = 0;

syms q0_wi_new q1_wi_new q2_wi_new q3_wi_new real;
q0_vw_new = 1; 
q1_vw_new = 0;
q2_vw_new = 0;
q3_vw_new = 0;

syms px_iw_old py_iw_old pz_iw_old real;
px_ci_old = 0;
py_ci_old = 0;
pz_ci_old = 0;
px_vw_old = 0;
py_vw_old = 0;
pz_vw_old = 0;

syms px_iw_new py_iw_new pz_iw_new real;
px_ci_new = 0;
py_ci_new = 0;
pz_ci_new = 0;
px_vw_new = 0;
py_vw_new = 0;
pz_vw_new = 0;

lamda_old = 1; 
lamda_new = 1;

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

z_p_relative = Pos_cv_new - Pos_cv_old;


H_z_p_relative_qwi_old = jacobian(z_p_relative,[q0_wi_old;q1_wi_old;q2_wi_old;q3_wi_old]);

H_z_p_relative_piw_old = jacobian(z_p_relative,[px_iw_old;py_iw_old;pz_iw_old]);
% 
% H_z_p_relative_lamda_old = jacobian(z_p_relative, lamda_old);
% 
% H_z_p_relative_pci_old = jacobian(z_p_relative, [px_ci_old;py_ci_old;pz_ci_old]);
% 
% H_z_p_relative_pvw_old = jacobian(z_p_relative, [px_vw_old;py_vw_old;pz_vw_old]);
% 
% H_z_p_relative_qvw_old = jacobian(z_p_relative, [q0_vw_old;q1_vw_old;q2_vw_old;q3_vw_old]);
% 
% 
% 
H_z_p_relative_qwi_new = jacobian(z_p_relative,[q0_wi_new;q1_wi_new;q2_wi_new;q3_wi_new]);
% 
H_z_p_relative_piw_new = jacobian(z_p_relative,[px_iw_new;py_iw_new;pz_iw_new]);
% 
% H_z_p_relative_lamda_new = jacobian(z_p_relative, lamda_new);
% 
% H_z_p_relative_pci_new = jacobian(z_p_relative, [px_ci_new;py_ci_new;pz_ci_new]);
% 
% H_z_p_relative_pvw_new = jacobian(z_p_relative, [px_vw_new;py_vw_new;pz_vw_new]);
% 
% H_z_p_relative_qvw_new = jacobian(z_p_relative, [q0_vw_new;q1_vw_new;q2_vw_new;q3_vw_new]);