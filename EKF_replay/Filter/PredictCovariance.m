function [P, F] = PredictCovariance( ...
    deltaAngle, ...
    deltaVelocity, ...
    states,...
    P, ...  % Previous state covariance matrix
    dt, ... % IMU and prediction time step
    param) % tuning parameters

% Set filter state process noise other than IMU errors, which are already
% built into the derived covariance predition equations.
% This process noise determines the rate of estimation of IMU bias errors
dAngBiasSigma = (dt*param.prediction.dGyrBiasPnoise)^2;
dVelBiasSigma = (dt*param.prediction.dAccBiasPnoise)^2;
magSigmaNED = dt*param.prediction.magPnoiseNED;
magSigmaXYZ = dt*param.prediction.magPnoiseXYZ;
lamdaSigma = dt*param.prediction.lamdaPnoise;
posSigmaCI = dt*param.prediction.posPnoiseCI;
quatSigmaIC = dt*param.prediction.quatPnoiseIC;
posSigmaVW = dt*param.prediction.posPnoiseVW;
quatSigmaVW = dt*param.prediction.quatPnoiseVW;
% New process noise state number is 39, 39*1
processNoiseVariance = [zeros(1,10), dAngBiasSigma*[1 1 1], dVelBiasSigma*[1 1 1], magSigmaNED*[1 1 1], magSigmaXYZ*[1 1 1], [0 0], lamdaSigma, posSigmaCI*[1 1 1], quatSigmaIC*[1 1 1 1], posSigmaVW*[1 1 1], quatSigmaVW*[1 1 1 1]].^2; % zeros(1,17)].^2;

% Specify the noise variance on the IMU delta angles and delta velocities
daxVar = (dt*param.prediction.angRateNoise)^2;
dayVar = daxVar;
dazVar = daxVar;
dvxVar = (dt*param.prediction.accelNoise)^2;
dvyVar = dvxVar;
dvzVar = dvxVar;

dvx = deltaVelocity(1);
dvy = deltaVelocity(2);
dvz = deltaVelocity(3);
dax = deltaAngle(1);
day = deltaAngle(2);
daz = deltaAngle(3);

q0 = states(1);
q1 = states(2);
q2 = states(3);
q3 = states(4);

dax_b = states(11);
day_b = states(12);
daz_b = states(13);

dvx_b = states(14);
dvy_b = states(15);
dvz_b = states(16);

% Predicted covariance
F = calcF39(dax,dax_b,day,day_b,daz,daz_b,dt,dvx,dvx_b,dvy,dvy_b,dvz,dvz_b,q0,q1,q2,q3);
Q = calcQ39(daxVar,dayVar,dazVar,dvxVar,dvyVar,dvzVar,q0,q1,q2,q3);
P = F*P*transpose(F) + Q;

% Add the general process noise variance
for i = 1:39
    P(i,i) = P(i,i) + processNoiseVariance(i);
end

% Force symmetry on the covariance matrix to prevent ill-conditioning
% of the matrix which would cause the filter to blow-up
P = 0.5*(P + transpose(P));

% ensure diagonals are positive
for i=1:39
    if P(i,i) < 0
        P(i,i) = 0;
    end
end

end