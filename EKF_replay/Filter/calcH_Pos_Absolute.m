function H_Pos_Absolute = calcH_Pos_Absolute(param,states)

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

H_pcv_qwi = [[ lamda*((2*q0_vw*q2_vw + 2*q1_vw*q3_vw)*(2*py_ci*q1_wi - 2*px_ci*q2_wi + 2*pz_ci*q0_wi) - (2*q0_vw*q3_vw - 2*q1_vw*q2_vw)*(2*px_ci*q3_wi + 2*py_ci*q0_wi - 2*pz_ci*q1_wi) + (2*px_ci*q0_wi - 2*py_ci*q3_wi + 2*pz_ci*q2_wi)*(q0_vw^2 + q1_vw^2 - q2_vw^2 - q3_vw^2)),  lamda*((2*q0_vw*q3_vw - 2*q1_vw*q2_vw)*(2*py_ci*q1_wi - 2*px_ci*q2_wi + 2*pz_ci*q0_wi) + (2*q0_vw*q2_vw + 2*q1_vw*q3_vw)*(2*px_ci*q3_wi + 2*py_ci*q0_wi - 2*pz_ci*q1_wi) + (2*px_ci*q1_wi + 2*py_ci*q2_wi + 2*pz_ci*q3_wi)*(q0_vw^2 + q1_vw^2 - q2_vw^2 - q3_vw^2)), -lamda*((2*q0_vw*q2_vw + 2*q1_vw*q3_vw)*(2*px_ci*q0_wi - 2*py_ci*q3_wi + 2*pz_ci*q2_wi) + (2*q0_vw*q3_vw - 2*q1_vw*q2_vw)*(2*px_ci*q1_wi + 2*py_ci*q2_wi + 2*pz_ci*q3_wi) - (2*py_ci*q1_wi - 2*px_ci*q2_wi + 2*pz_ci*q0_wi)*(q0_vw^2 + q1_vw^2 - q2_vw^2 - q3_vw^2)), -lamda*((2*q0_vw*q3_vw - 2*q1_vw*q2_vw)*(2*px_ci*q0_wi - 2*py_ci*q3_wi + 2*pz_ci*q2_wi) - (2*q0_vw*q2_vw + 2*q1_vw*q3_vw)*(2*px_ci*q1_wi + 2*py_ci*q2_wi + 2*pz_ci*q3_wi) + (2*px_ci*q3_wi + 2*py_ci*q0_wi - 2*pz_ci*q1_wi)*(q0_vw^2 + q1_vw^2 - q2_vw^2 - q3_vw^2))];
             [ lamda*((2*q0_vw*q3_vw + 2*q1_vw*q2_vw)*(2*px_ci*q0_wi - 2*py_ci*q3_wi + 2*pz_ci*q2_wi) - (2*q0_vw*q1_vw - 2*q2_vw*q3_vw)*(2*py_ci*q1_wi - 2*px_ci*q2_wi + 2*pz_ci*q0_wi) + (2*px_ci*q3_wi + 2*py_ci*q0_wi - 2*pz_ci*q1_wi)*(q0_vw^2 - q1_vw^2 + q2_vw^2 - q3_vw^2)), -lamda*((2*q0_vw*q1_vw - 2*q2_vw*q3_vw)*(2*px_ci*q3_wi + 2*py_ci*q0_wi - 2*pz_ci*q1_wi) - (2*q0_vw*q3_vw + 2*q1_vw*q2_vw)*(2*px_ci*q1_wi + 2*py_ci*q2_wi + 2*pz_ci*q3_wi) + (2*py_ci*q1_wi - 2*px_ci*q2_wi + 2*pz_ci*q0_wi)*(q0_vw^2 - q1_vw^2 + q2_vw^2 - q3_vw^2)),  lamda*((2*q0_vw*q3_vw + 2*q1_vw*q2_vw)*(2*py_ci*q1_wi - 2*px_ci*q2_wi + 2*pz_ci*q0_wi) + (2*q0_vw*q1_vw - 2*q2_vw*q3_vw)*(2*px_ci*q0_wi - 2*py_ci*q3_wi + 2*pz_ci*q2_wi) + (2*px_ci*q1_wi + 2*py_ci*q2_wi + 2*pz_ci*q3_wi)*(q0_vw^2 - q1_vw^2 + q2_vw^2 - q3_vw^2)), -lamda*((2*q0_vw*q3_vw + 2*q1_vw*q2_vw)*(2*px_ci*q3_wi + 2*py_ci*q0_wi - 2*pz_ci*q1_wi) + (2*q0_vw*q1_vw - 2*q2_vw*q3_vw)*(2*px_ci*q1_wi + 2*py_ci*q2_wi + 2*pz_ci*q3_wi) - (2*px_ci*q0_wi - 2*py_ci*q3_wi + 2*pz_ci*q2_wi)*(q0_vw^2 - q1_vw^2 + q2_vw^2 - q3_vw^2))];
             [ lamda*((2*q0_vw*q1_vw + 2*q2_vw*q3_vw)*(2*px_ci*q3_wi + 2*py_ci*q0_wi - 2*pz_ci*q1_wi) - (2*q0_vw*q2_vw - 2*q1_vw*q3_vw)*(2*px_ci*q0_wi - 2*py_ci*q3_wi + 2*pz_ci*q2_wi) + (2*py_ci*q1_wi - 2*px_ci*q2_wi + 2*pz_ci*q0_wi)*(q0_vw^2 - q1_vw^2 - q2_vw^2 + q3_vw^2)), -lamda*((2*q0_vw*q1_vw + 2*q2_vw*q3_vw)*(2*py_ci*q1_wi - 2*px_ci*q2_wi + 2*pz_ci*q0_wi) + (2*q0_vw*q2_vw - 2*q1_vw*q3_vw)*(2*px_ci*q1_wi + 2*py_ci*q2_wi + 2*pz_ci*q3_wi) - (2*px_ci*q3_wi + 2*py_ci*q0_wi - 2*pz_ci*q1_wi)*(q0_vw^2 - q1_vw^2 - q2_vw^2 + q3_vw^2)), -lamda*((2*q0_vw*q2_vw - 2*q1_vw*q3_vw)*(2*py_ci*q1_wi - 2*px_ci*q2_wi + 2*pz_ci*q0_wi) - (2*q0_vw*q1_vw + 2*q2_vw*q3_vw)*(2*px_ci*q1_wi + 2*py_ci*q2_wi + 2*pz_ci*q3_wi) + (2*px_ci*q0_wi - 2*py_ci*q3_wi + 2*pz_ci*q2_wi)*(q0_vw^2 - q1_vw^2 - q2_vw^2 + q3_vw^2)),  lamda*((2*q0_vw*q2_vw - 2*q1_vw*q3_vw)*(2*px_ci*q3_wi + 2*py_ci*q0_wi - 2*pz_ci*q1_wi) + (2*q0_vw*q1_vw + 2*q2_vw*q3_vw)*(2*px_ci*q0_wi - 2*py_ci*q3_wi + 2*pz_ci*q2_wi) + (2*px_ci*q1_wi + 2*py_ci*q2_wi + 2*pz_ci*q3_wi)*(q0_vw^2 - q1_vw^2 - q2_vw^2 + q3_vw^2))]];

H_pcv_piw = [[ lamda*(q0_vw^2 + q1_vw^2 - q2_vw^2 - q3_vw^2),        -lamda*(2*q0_vw*q3_vw - 2*q1_vw*q2_vw),         lamda*(2*q0_vw*q2_vw + 2*q1_vw*q3_vw)]
             [         lamda*(2*q0_vw*q3_vw + 2*q1_vw*q2_vw), lamda*(q0_vw^2 - q1_vw^2 + q2_vw^2 - q3_vw^2),        -lamda*(2*q0_vw*q1_vw - 2*q2_vw*q3_vw)]
             [        -lamda*(2*q0_vw*q2_vw - 2*q1_vw*q3_vw),         lamda*(2*q0_vw*q1_vw + 2*q2_vw*q3_vw), lamda*(q0_vw^2 - q1_vw^2 - q2_vw^2 + q3_vw^2)]];

if (param.fusion.camFixScale == 0)
    H_pcv_lamda = [(2*q0_vw*q2_vw + 2*q1_vw*q3_vw)*(pz_iw - pz_vw + pz_ci*(q0_wi^2 - q1_wi^2 - q2_wi^2 + q3_wi^2) - px_ci*(2*q0_wi*q2_wi - 2*q1_wi*q3_wi) + py_ci*(2*q0_wi*q1_wi + 2*q2_wi*q3_wi)) - (2*q0_vw*q3_vw - 2*q1_vw*q2_vw)*(py_iw - py_vw + py_ci*(q0_wi^2 - q1_wi^2 + q2_wi^2 - q3_wi^2) + px_ci*(2*q0_wi*q3_wi + 2*q1_wi*q2_wi) - pz_ci*(2*q0_wi*q1_wi - 2*q2_wi*q3_wi)) + (q0_vw^2 + q1_vw^2 - q2_vw^2 - q3_vw^2)*(px_iw - px_vw + px_ci*(q0_wi^2 + q1_wi^2 - q2_wi^2 - q3_wi^2) - py_ci*(2*q0_wi*q3_wi - 2*q1_wi*q2_wi) + pz_ci*(2*q0_wi*q2_wi + 2*q1_wi*q3_wi));
                   (2*q0_vw*q3_vw + 2*q1_vw*q2_vw)*(px_iw - px_vw + px_ci*(q0_wi^2 + q1_wi^2 - q2_wi^2 - q3_wi^2) - py_ci*(2*q0_wi*q3_wi - 2*q1_wi*q2_wi) + pz_ci*(2*q0_wi*q2_wi + 2*q1_wi*q3_wi)) - (2*q0_vw*q1_vw - 2*q2_vw*q3_vw)*(pz_iw - pz_vw + pz_ci*(q0_wi^2 - q1_wi^2 - q2_wi^2 + q3_wi^2) - px_ci*(2*q0_wi*q2_wi - 2*q1_wi*q3_wi) + py_ci*(2*q0_wi*q1_wi + 2*q2_wi*q3_wi)) + (q0_vw^2 - q1_vw^2 + q2_vw^2 - q3_vw^2)*(py_iw - py_vw + py_ci*(q0_wi^2 - q1_wi^2 + q2_wi^2 - q3_wi^2) + px_ci*(2*q0_wi*q3_wi + 2*q1_wi*q2_wi) - pz_ci*(2*q0_wi*q1_wi - 2*q2_wi*q3_wi));
                   (2*q0_vw*q1_vw + 2*q2_vw*q3_vw)*(py_iw - py_vw + py_ci*(q0_wi^2 - q1_wi^2 + q2_wi^2 - q3_wi^2) + px_ci*(2*q0_wi*q3_wi + 2*q1_wi*q2_wi) - pz_ci*(2*q0_wi*q1_wi - 2*q2_wi*q3_wi)) - (2*q0_vw*q2_vw - 2*q1_vw*q3_vw)*(px_iw - px_vw + px_ci*(q0_wi^2 + q1_wi^2 - q2_wi^2 - q3_wi^2) - py_ci*(2*q0_wi*q3_wi - 2*q1_wi*q2_wi) + pz_ci*(2*q0_wi*q2_wi + 2*q1_wi*q3_wi)) + (q0_vw^2 - q1_vw^2 - q2_vw^2 + q3_vw^2)*(pz_iw - pz_vw + pz_ci*(q0_wi^2 - q1_wi^2 - q2_wi^2 + q3_wi^2) - px_ci*(2*q0_wi*q2_wi - 2*q1_wi*q3_wi) + py_ci*(2*q0_wi*q1_wi + 2*q2_wi*q3_wi))];
                
elseif (param.fusion.camFixScale == 1)
    H_pcv_lamda = zeros(3,1);
                
end

if (param.fusion.camFixPosci == 0)
    H_pcv_pci = [[ -lamda*((2*q0_wi*q2_wi - 2*q1_wi*q3_wi)*(2*q0_vw*q2_vw + 2*q1_vw*q3_vw) - (q0_wi^2 + q1_wi^2 - q2_wi^2 - q3_wi^2)*(q0_vw^2 + q1_vw^2 - q2_vw^2 - q3_vw^2) + (2*q0_wi*q3_wi + 2*q1_wi*q2_wi)*(2*q0_vw*q3_vw - 2*q1_vw*q2_vw)), -lamda*((2*q0_wi*q3_wi - 2*q1_wi*q2_wi)*(q0_vw^2 + q1_vw^2 - q2_vw^2 - q3_vw^2) - (2*q0_wi*q1_wi + 2*q2_wi*q3_wi)*(2*q0_vw*q2_vw + 2*q1_vw*q3_vw) + (2*q0_vw*q3_vw - 2*q1_vw*q2_vw)*(q0_wi^2 - q1_wi^2 + q2_wi^2 - q3_wi^2)),  lamda*((2*q0_wi*q1_wi - 2*q2_wi*q3_wi)*(2*q0_vw*q3_vw - 2*q1_vw*q2_vw) + (2*q0_wi*q2_wi + 2*q1_wi*q3_wi)*(q0_vw^2 + q1_vw^2 - q2_vw^2 - q3_vw^2) + (2*q0_vw*q2_vw + 2*q1_vw*q3_vw)*(q0_wi^2 - q1_wi^2 - q2_wi^2 + q3_wi^2))]
                 [  lamda*((2*q0_wi*q2_wi - 2*q1_wi*q3_wi)*(2*q0_vw*q1_vw - 2*q2_vw*q3_vw) + (2*q0_wi*q3_wi + 2*q1_wi*q2_wi)*(q0_vw^2 - q1_vw^2 + q2_vw^2 - q3_vw^2) + (2*q0_vw*q3_vw + 2*q1_vw*q2_vw)*(q0_wi^2 + q1_wi^2 - q2_wi^2 - q3_wi^2)), -lamda*((2*q0_wi*q1_wi + 2*q2_wi*q3_wi)*(2*q0_vw*q1_vw - 2*q2_vw*q3_vw) - (q0_wi^2 - q1_wi^2 + q2_wi^2 - q3_wi^2)*(q0_vw^2 - q1_vw^2 + q2_vw^2 - q3_vw^2) + (2*q0_wi*q3_wi - 2*q1_wi*q2_wi)*(2*q0_vw*q3_vw + 2*q1_vw*q2_vw)), -lamda*((2*q0_wi*q1_wi - 2*q2_wi*q3_wi)*(q0_vw^2 - q1_vw^2 + q2_vw^2 - q3_vw^2) - (2*q0_wi*q2_wi + 2*q1_wi*q3_wi)*(2*q0_vw*q3_vw + 2*q1_vw*q2_vw) + (2*q0_vw*q1_vw - 2*q2_vw*q3_vw)*(q0_wi^2 - q1_wi^2 - q2_wi^2 + q3_wi^2))]
                 [ -lamda*((2*q0_wi*q2_wi - 2*q1_wi*q3_wi)*(q0_vw^2 - q1_vw^2 - q2_vw^2 + q3_vw^2) - (2*q0_wi*q3_wi + 2*q1_wi*q2_wi)*(2*q0_vw*q1_vw + 2*q2_vw*q3_vw) + (2*q0_vw*q2_vw - 2*q1_vw*q3_vw)*(q0_wi^2 + q1_wi^2 - q2_wi^2 - q3_wi^2)),  lamda*((2*q0_wi*q3_wi - 2*q1_wi*q2_wi)*(2*q0_vw*q2_vw - 2*q1_vw*q3_vw) + (2*q0_wi*q1_wi + 2*q2_wi*q3_wi)*(q0_vw^2 - q1_vw^2 - q2_vw^2 + q3_vw^2) + (2*q0_vw*q1_vw + 2*q2_vw*q3_vw)*(q0_wi^2 - q1_wi^2 + q2_wi^2 - q3_wi^2)), -lamda*((2*q0_wi*q1_wi - 2*q2_wi*q3_wi)*(2*q0_vw*q1_vw + 2*q2_vw*q3_vw) - (q0_wi^2 - q1_wi^2 - q2_wi^2 + q3_wi^2)*(q0_vw^2 - q1_vw^2 - q2_vw^2 + q3_vw^2) + (2*q0_wi*q2_wi + 2*q1_wi*q3_wi)*(2*q0_vw*q2_vw - 2*q1_vw*q3_vw))]];
                
elseif (param.fusion.camFixPosci == 1)
    H_pcv_pci = zeros(3,3);
                
end

if (param.fusion.camFixPosvw == 0)
    H_pcv_pvw = [[ -lamda*(q0_vw^2 + q1_vw^2 - q2_vw^2 - q3_vw^2),          lamda*(2*q0_vw*q3_vw - 2*q1_vw*q2_vw),         -lamda*(2*q0_vw*q2_vw + 2*q1_vw*q3_vw)]
                 [         -lamda*(2*q0_vw*q3_vw + 2*q1_vw*q2_vw), -lamda*(q0_vw^2 - q1_vw^2 + q2_vw^2 - q3_vw^2),          lamda*(2*q0_vw*q1_vw - 2*q2_vw*q3_vw)]
                 [          lamda*(2*q0_vw*q2_vw - 2*q1_vw*q3_vw),         -lamda*(2*q0_vw*q1_vw + 2*q2_vw*q3_vw), -lamda*(q0_vw^2 - q1_vw^2 - q2_vw^2 + q3_vw^2)]];
                
elseif (param.fusion.camFixPosvw == 1)
    H_pcv_pvw = zeros(3,3);
                
end

if (param.fusion.camFixQuatvw == 0)
    H_pcv_qvw = [[ lamda*(2*q0_vw*(px_iw - px_vw + px_ci*(q0_wi^2 + q1_wi^2 - q2_wi^2 - q3_wi^2) - py_ci*(2*q0_wi*q3_wi - 2*q1_wi*q2_wi) + pz_ci*(2*q0_wi*q2_wi + 2*q1_wi*q3_wi)) - 2*q3_vw*(py_iw - py_vw + py_ci*(q0_wi^2 - q1_wi^2 + q2_wi^2 - q3_wi^2) + px_ci*(2*q0_wi*q3_wi + 2*q1_wi*q2_wi) - pz_ci*(2*q0_wi*q1_wi - 2*q2_wi*q3_wi)) + 2*q2_vw*(pz_iw - pz_vw + pz_ci*(q0_wi^2 - q1_wi^2 - q2_wi^2 + q3_wi^2) - px_ci*(2*q0_wi*q2_wi - 2*q1_wi*q3_wi) + py_ci*(2*q0_wi*q1_wi + 2*q2_wi*q3_wi))),  lamda*(2*q1_vw*(px_iw - px_vw + px_ci*(q0_wi^2 + q1_wi^2 - q2_wi^2 - q3_wi^2) - py_ci*(2*q0_wi*q3_wi - 2*q1_wi*q2_wi) + pz_ci*(2*q0_wi*q2_wi + 2*q1_wi*q3_wi)) + 2*q2_vw*(py_iw - py_vw + py_ci*(q0_wi^2 - q1_wi^2 + q2_wi^2 - q3_wi^2) + px_ci*(2*q0_wi*q3_wi + 2*q1_wi*q2_wi) - pz_ci*(2*q0_wi*q1_wi - 2*q2_wi*q3_wi)) + 2*q3_vw*(pz_iw - pz_vw + pz_ci*(q0_wi^2 - q1_wi^2 - q2_wi^2 + q3_wi^2) - px_ci*(2*q0_wi*q2_wi - 2*q1_wi*q3_wi) + py_ci*(2*q0_wi*q1_wi + 2*q2_wi*q3_wi))),  lamda*(2*q1_vw*(py_iw - py_vw + py_ci*(q0_wi^2 - q1_wi^2 + q2_wi^2 - q3_wi^2) + px_ci*(2*q0_wi*q3_wi + 2*q1_wi*q2_wi) - pz_ci*(2*q0_wi*q1_wi - 2*q2_wi*q3_wi)) - 2*q2_vw*(px_iw - px_vw + px_ci*(q0_wi^2 + q1_wi^2 - q2_wi^2 - q3_wi^2) - py_ci*(2*q0_wi*q3_wi - 2*q1_wi*q2_wi) + pz_ci*(2*q0_wi*q2_wi + 2*q1_wi*q3_wi)) + 2*q0_vw*(pz_iw - pz_vw + pz_ci*(q0_wi^2 - q1_wi^2 - q2_wi^2 + q3_wi^2) - px_ci*(2*q0_wi*q2_wi - 2*q1_wi*q3_wi) + py_ci*(2*q0_wi*q1_wi + 2*q2_wi*q3_wi))), -lamda*(2*q3_vw*(px_iw - px_vw + px_ci*(q0_wi^2 + q1_wi^2 - q2_wi^2 - q3_wi^2) - py_ci*(2*q0_wi*q3_wi - 2*q1_wi*q2_wi) + pz_ci*(2*q0_wi*q2_wi + 2*q1_wi*q3_wi)) + 2*q0_vw*(py_iw - py_vw + py_ci*(q0_wi^2 - q1_wi^2 + q2_wi^2 - q3_wi^2) + px_ci*(2*q0_wi*q3_wi + 2*q1_wi*q2_wi) - pz_ci*(2*q0_wi*q1_wi - 2*q2_wi*q3_wi)) - 2*q1_vw*(pz_iw - pz_vw + pz_ci*(q0_wi^2 - q1_wi^2 - q2_wi^2 + q3_wi^2) - px_ci*(2*q0_wi*q2_wi - 2*q1_wi*q3_wi) + py_ci*(2*q0_wi*q1_wi + 2*q2_wi*q3_wi)))]
                 [ lamda*(2*q3_vw*(px_iw - px_vw + px_ci*(q0_wi^2 + q1_wi^2 - q2_wi^2 - q3_wi^2) - py_ci*(2*q0_wi*q3_wi - 2*q1_wi*q2_wi) + pz_ci*(2*q0_wi*q2_wi + 2*q1_wi*q3_wi)) + 2*q0_vw*(py_iw - py_vw + py_ci*(q0_wi^2 - q1_wi^2 + q2_wi^2 - q3_wi^2) + px_ci*(2*q0_wi*q3_wi + 2*q1_wi*q2_wi) - pz_ci*(2*q0_wi*q1_wi - 2*q2_wi*q3_wi)) - 2*q1_vw*(pz_iw - pz_vw + pz_ci*(q0_wi^2 - q1_wi^2 - q2_wi^2 + q3_wi^2) - px_ci*(2*q0_wi*q2_wi - 2*q1_wi*q3_wi) + py_ci*(2*q0_wi*q1_wi + 2*q2_wi*q3_wi))), -lamda*(2*q1_vw*(py_iw - py_vw + py_ci*(q0_wi^2 - q1_wi^2 + q2_wi^2 - q3_wi^2) + px_ci*(2*q0_wi*q3_wi + 2*q1_wi*q2_wi) - pz_ci*(2*q0_wi*q1_wi - 2*q2_wi*q3_wi)) - 2*q2_vw*(px_iw - px_vw + px_ci*(q0_wi^2 + q1_wi^2 - q2_wi^2 - q3_wi^2) - py_ci*(2*q0_wi*q3_wi - 2*q1_wi*q2_wi) + pz_ci*(2*q0_wi*q2_wi + 2*q1_wi*q3_wi)) + 2*q0_vw*(pz_iw - pz_vw + pz_ci*(q0_wi^2 - q1_wi^2 - q2_wi^2 + q3_wi^2) - px_ci*(2*q0_wi*q2_wi - 2*q1_wi*q3_wi) + py_ci*(2*q0_wi*q1_wi + 2*q2_wi*q3_wi))),  lamda*(2*q1_vw*(px_iw - px_vw + px_ci*(q0_wi^2 + q1_wi^2 - q2_wi^2 - q3_wi^2) - py_ci*(2*q0_wi*q3_wi - 2*q1_wi*q2_wi) + pz_ci*(2*q0_wi*q2_wi + 2*q1_wi*q3_wi)) + 2*q2_vw*(py_iw - py_vw + py_ci*(q0_wi^2 - q1_wi^2 + q2_wi^2 - q3_wi^2) + px_ci*(2*q0_wi*q3_wi + 2*q1_wi*q2_wi) - pz_ci*(2*q0_wi*q1_wi - 2*q2_wi*q3_wi)) + 2*q3_vw*(pz_iw - pz_vw + pz_ci*(q0_wi^2 - q1_wi^2 - q2_wi^2 + q3_wi^2) - px_ci*(2*q0_wi*q2_wi - 2*q1_wi*q3_wi) + py_ci*(2*q0_wi*q1_wi + 2*q2_wi*q3_wi))),  lamda*(2*q0_vw*(px_iw - px_vw + px_ci*(q0_wi^2 + q1_wi^2 - q2_wi^2 - q3_wi^2) - py_ci*(2*q0_wi*q3_wi - 2*q1_wi*q2_wi) + pz_ci*(2*q0_wi*q2_wi + 2*q1_wi*q3_wi)) - 2*q3_vw*(py_iw - py_vw + py_ci*(q0_wi^2 - q1_wi^2 + q2_wi^2 - q3_wi^2) + px_ci*(2*q0_wi*q3_wi + 2*q1_wi*q2_wi) - pz_ci*(2*q0_wi*q1_wi - 2*q2_wi*q3_wi)) + 2*q2_vw*(pz_iw - pz_vw + pz_ci*(q0_wi^2 - q1_wi^2 - q2_wi^2 + q3_wi^2) - px_ci*(2*q0_wi*q2_wi - 2*q1_wi*q3_wi) + py_ci*(2*q0_wi*q1_wi + 2*q2_wi*q3_wi)))]
                 [ lamda*(2*q1_vw*(py_iw - py_vw + py_ci*(q0_wi^2 - q1_wi^2 + q2_wi^2 - q3_wi^2) + px_ci*(2*q0_wi*q3_wi + 2*q1_wi*q2_wi) - pz_ci*(2*q0_wi*q1_wi - 2*q2_wi*q3_wi)) - 2*q2_vw*(px_iw - px_vw + px_ci*(q0_wi^2 + q1_wi^2 - q2_wi^2 - q3_wi^2) - py_ci*(2*q0_wi*q3_wi - 2*q1_wi*q2_wi) + pz_ci*(2*q0_wi*q2_wi + 2*q1_wi*q3_wi)) + 2*q0_vw*(pz_iw - pz_vw + pz_ci*(q0_wi^2 - q1_wi^2 - q2_wi^2 + q3_wi^2) - px_ci*(2*q0_wi*q2_wi - 2*q1_wi*q3_wi) + py_ci*(2*q0_wi*q1_wi + 2*q2_wi*q3_wi))),  lamda*(2*q3_vw*(px_iw - px_vw + px_ci*(q0_wi^2 + q1_wi^2 - q2_wi^2 - q3_wi^2) - py_ci*(2*q0_wi*q3_wi - 2*q1_wi*q2_wi) + pz_ci*(2*q0_wi*q2_wi + 2*q1_wi*q3_wi)) + 2*q0_vw*(py_iw - py_vw + py_ci*(q0_wi^2 - q1_wi^2 + q2_wi^2 - q3_wi^2) + px_ci*(2*q0_wi*q3_wi + 2*q1_wi*q2_wi) - pz_ci*(2*q0_wi*q1_wi - 2*q2_wi*q3_wi)) - 2*q1_vw*(pz_iw - pz_vw + pz_ci*(q0_wi^2 - q1_wi^2 - q2_wi^2 + q3_wi^2) - px_ci*(2*q0_wi*q2_wi - 2*q1_wi*q3_wi) + py_ci*(2*q0_wi*q1_wi + 2*q2_wi*q3_wi))), -lamda*(2*q0_vw*(px_iw - px_vw + px_ci*(q0_wi^2 + q1_wi^2 - q2_wi^2 - q3_wi^2) - py_ci*(2*q0_wi*q3_wi - 2*q1_wi*q2_wi) + pz_ci*(2*q0_wi*q2_wi + 2*q1_wi*q3_wi)) - 2*q3_vw*(py_iw - py_vw + py_ci*(q0_wi^2 - q1_wi^2 + q2_wi^2 - q3_wi^2) + px_ci*(2*q0_wi*q3_wi + 2*q1_wi*q2_wi) - pz_ci*(2*q0_wi*q1_wi - 2*q2_wi*q3_wi)) + 2*q2_vw*(pz_iw - pz_vw + pz_ci*(q0_wi^2 - q1_wi^2 - q2_wi^2 + q3_wi^2) - px_ci*(2*q0_wi*q2_wi - 2*q1_wi*q3_wi) + py_ci*(2*q0_wi*q1_wi + 2*q2_wi*q3_wi))),  lamda*(2*q1_vw*(px_iw - px_vw + px_ci*(q0_wi^2 + q1_wi^2 - q2_wi^2 - q3_wi^2) - py_ci*(2*q0_wi*q3_wi - 2*q1_wi*q2_wi) + pz_ci*(2*q0_wi*q2_wi + 2*q1_wi*q3_wi)) + 2*q2_vw*(py_iw - py_vw + py_ci*(q0_wi^2 - q1_wi^2 + q2_wi^2 - q3_wi^2) + px_ci*(2*q0_wi*q3_wi + 2*q1_wi*q2_wi) - pz_ci*(2*q0_wi*q1_wi - 2*q2_wi*q3_wi)) + 2*q3_vw*(pz_iw - pz_vw + pz_ci*(q0_wi^2 - q1_wi^2 - q2_wi^2 + q3_wi^2) - px_ci*(2*q0_wi*q2_wi - 2*q1_wi*q3_wi) + py_ci*(2*q0_wi*q1_wi + 2*q2_wi*q3_wi)))]];
                
elseif (param.fusion.camFixQuatvw == 1)
    H_pcv_qvw = zeros(3,4);
                
end

H_Pos_Absolute = [H_pcv_qwi, zeros(3,3), H_pcv_piw, zeros(3,14), H_pcv_lamda, H_pcv_pci, zeros(3,4), H_pcv_pvw, H_pcv_qvw];
