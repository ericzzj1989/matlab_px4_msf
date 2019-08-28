function H_Att_Absolute = calcH_Att_Absolute(param,states)

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

H_qvc_qwi = [[ q0_ic*q0_vw - q1_ic*q1_vw - q2_ic*q2_vw - q3_ic*q3_vw, q3_ic*q2_vw - q1_ic*q0_vw - q2_ic*q3_vw - q0_ic*q1_vw, q1_ic*q3_vw - q2_ic*q0_vw - q0_ic*q2_vw - q3_ic*q1_vw, q2_ic*q1_vw - q1_ic*q2_vw - q0_ic*q3_vw - q3_ic*q0_vw]
             [ q0_ic*q1_vw + q1_ic*q0_vw - q2_ic*q3_vw + q3_ic*q2_vw, q0_ic*q0_vw - q1_ic*q1_vw + q2_ic*q2_vw + q3_ic*q3_vw, q3_ic*q0_vw - q1_ic*q2_vw - q2_ic*q1_vw - q0_ic*q3_vw, q0_ic*q2_vw - q2_ic*q0_vw - q1_ic*q3_vw - q3_ic*q1_vw]
             [ q0_ic*q2_vw + q2_ic*q0_vw + q1_ic*q3_vw - q3_ic*q1_vw, q0_ic*q3_vw - q1_ic*q2_vw - q2_ic*q1_vw - q3_ic*q0_vw, q0_ic*q0_vw + q1_ic*q1_vw - q2_ic*q2_vw + q3_ic*q3_vw, q1_ic*q0_vw - q0_ic*q1_vw - q2_ic*q3_vw - q3_ic*q2_vw]
             [ q0_ic*q3_vw - q1_ic*q2_vw + q2_ic*q1_vw + q3_ic*q0_vw, q2_ic*q0_vw - q0_ic*q2_vw - q1_ic*q3_vw - q3_ic*q1_vw, q0_ic*q1_vw - q1_ic*q0_vw - q2_ic*q3_vw - q3_ic*q2_vw, q0_ic*q0_vw + q1_ic*q1_vw + q2_ic*q2_vw - q3_ic*q3_vw]];

if (param.fusion.camFixQuatic == 0)
    H_qvc_qic = [[ q0_wi*q0_vw - q1_wi*q1_vw - q2_wi*q2_vw - q3_wi*q3_vw, q2_wi*q3_vw - q1_wi*q0_vw - q0_wi*q1_vw - q3_wi*q2_vw, q3_wi*q1_vw - q2_wi*q0_vw - q1_wi*q3_vw - q0_wi*q2_vw, q1_wi*q2_vw - q0_wi*q3_vw - q2_wi*q1_vw - q3_wi*q0_vw]
                 [ q0_wi*q1_vw + q1_wi*q0_vw - q2_wi*q3_vw + q3_wi*q2_vw, q0_wi*q0_vw - q1_wi*q1_vw - q2_wi*q2_vw - q3_wi*q3_vw, q1_wi*q2_vw - q0_wi*q3_vw - q2_wi*q1_vw - q3_wi*q0_vw, q0_wi*q2_vw + q2_wi*q0_vw + q1_wi*q3_vw - q3_wi*q1_vw]
                 [ q0_wi*q2_vw + q2_wi*q0_vw + q1_wi*q3_vw - q3_wi*q1_vw, q0_wi*q3_vw - q1_wi*q2_vw + q2_wi*q1_vw + q3_wi*q0_vw, q0_wi*q0_vw - q1_wi*q1_vw - q2_wi*q2_vw - q3_wi*q3_vw, q2_wi*q3_vw - q1_wi*q0_vw - q0_wi*q1_vw - q3_wi*q2_vw]
                 [ q0_wi*q3_vw - q1_wi*q2_vw + q2_wi*q1_vw + q3_wi*q0_vw, q3_wi*q1_vw - q2_wi*q0_vw - q1_wi*q3_vw - q0_wi*q2_vw, q0_wi*q1_vw + q1_wi*q0_vw - q2_wi*q3_vw + q3_wi*q2_vw, q0_wi*q0_vw - q1_wi*q1_vw - q2_wi*q2_vw - q3_wi*q3_vw]];
                
elseif (param.fusion.camFixQuatic == 1)
    H_qvc_qic = zeros(4,4);
                
end

if (param.fusion.camFixQuatvw == 0)
    H_qvc_qvw = [[ q0_ic*q0_wi - q1_ic*q1_wi - q2_ic*q2_wi - q3_ic*q3_wi, q2_ic*q3_wi - q1_ic*q0_wi - q0_ic*q1_wi - q3_ic*q2_wi, q3_ic*q1_wi - q2_ic*q0_wi - q1_ic*q3_wi - q0_ic*q2_wi, q1_ic*q2_wi - q0_ic*q3_wi - q2_ic*q1_wi - q3_ic*q0_wi]
                 [ q0_ic*q1_wi + q1_ic*q0_wi - q2_ic*q3_wi + q3_ic*q2_wi, q0_ic*q0_wi - q1_ic*q1_wi - q2_ic*q2_wi - q3_ic*q3_wi, q0_ic*q3_wi - q1_ic*q2_wi + q2_ic*q1_wi + q3_ic*q0_wi, q3_ic*q1_wi - q2_ic*q0_wi - q1_ic*q3_wi - q0_ic*q2_wi]
                 [ q0_ic*q2_wi + q2_ic*q0_wi + q1_ic*q3_wi - q3_ic*q1_wi, q1_ic*q2_wi - q0_ic*q3_wi - q2_ic*q1_wi - q3_ic*q0_wi, q0_ic*q0_wi - q1_ic*q1_wi - q2_ic*q2_wi - q3_ic*q3_wi, q0_ic*q1_wi + q1_ic*q0_wi - q2_ic*q3_wi + q3_ic*q2_wi]
                 [ q0_ic*q3_wi - q1_ic*q2_wi + q2_ic*q1_wi + q3_ic*q0_wi, q0_ic*q2_wi + q2_ic*q0_wi + q1_ic*q3_wi - q3_ic*q1_wi, q2_ic*q3_wi - q1_ic*q0_wi - q0_ic*q1_wi - q3_ic*q2_wi, q0_ic*q0_wi - q1_ic*q1_wi - q2_ic*q2_wi - q3_ic*q3_wi]];
                
elseif (param.fusion.camFixQuatvw == 1)
    H_qvc_qvw = zeros(4,4);
                
end

H_Att_Absolute = [H_qvc_qwi, zeros(4,24), H_qvc_qic, zeros(4,3), H_qvc_qvw];


