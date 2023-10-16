function [uCurr,covar_curr] = upd_step(z_t,covarEst,uEst)
%z_t is the measurement
%covarEst and uEst are the predicted covariance and mean respectively
%uCurr and covar_curr are the updated mean and covariance respectively


C = [eye(3) zeros(3,3) zeros(3,3) zeros(3,3) zeros(3,3);
     zeros(3,3) eye(3) zeros(3,3) zeros(3,3) zeros(3,3)];

v = zeros(6,1);
z = C*uEst + v;
Ct = C;
Wt = eye(6);
R = eye(6)*0.1;

Kt = covarEst*Ct'/(Ct*covarEst*Ct' + Wt*R*Wt');


uCurr = uEst + Kt*(z_t - z);
covar_curr = covarEst - Kt*Ct*covarEst;


end