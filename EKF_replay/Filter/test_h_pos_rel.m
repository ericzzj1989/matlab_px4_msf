addpath('../Common');

syms q0_wi_old q1_wi_old q2_wi_old q3_wi_old real;
syms q0_vw_old q1_vw_old q2_vw_old q3_vw_old real;

syms q0_wi_new q1_wi_new q2_wi_new q3_wi_new real;
syms q0_vw_new q1_vw_new q2_vw_new q3_vw_new real;

syms px_iw_old py_iw_old pz_iw_old real;
syms px_ci_old py_ci_old pz_ci_old real;
syms px_vw_old py_vw_old pz_vw_old real;

syms px_iw_new py_iw_new pz_iw_new real;
syms px_ci_new py_ci_new pz_ci_new real;
syms px_vw_new py_vw_new pz_vw_new real;

syms lamda_old lamda_new real;

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

H_z_p_relative_lamda_old = jacobian(z_p_relative, lamda_old);

H_z_p_relative_pci_old = jacobian(z_p_relative, [px_ci_old;py_ci_old;pz_ci_old]);

H_z_p_relative_pvw_old = jacobian(z_p_relative, [px_vw_old;py_vw_old;pz_vw_old]);

H_z_p_relative_qvw_old = jacobian(z_p_relative, [q0_vw_old;q1_vw_old;q2_vw_old;q3_vw_old]);



H_z_p_relative_qwi_new = jacobian(z_p_relative,[q0_wi_new;q1_wi_new;q2_wi_new;q3_wi_new]);

H_z_p_relative_piw_new = jacobian(z_p_relative,[px_iw_new;py_iw_new;pz_iw_new]);

H_z_p_relative_lamda_new = jacobian(z_p_relative, lamda_new);

H_z_p_relative_pci_new = jacobian(z_p_relative, [px_ci_new;py_ci_new;pz_ci_new]);

H_z_p_relative_pvw_new = jacobian(z_p_relative, [px_vw_new;py_vw_new;pz_vw_new]);

H_z_p_relative_qvw_new = jacobian(z_p_relative, [q0_vw_new;q1_vw_new;q2_vw_new;q3_vw_new]);

