 function [covarEst,uEst] = pred_step(uPrev,covarPrev,angVel,acc,dt)
%covarPrev and uPrev are the previous mean and covariance respectively
%angVel is the angular velocity
%acc is the acceleration
%dt is the sampling time

p_dot_x = uPrev(7,1);
p_dot_y = uPrev(8,1);
p_dot_z = uPrev(9,1);

omega_m_x = angVel(1,1);
omega_m_y = angVel(2,1);
omega_m_z = angVel(3,1);

acc_m_x = acc(1,1);
acc_m_y = acc(2,1);
acc_m_z = acc(3,1);

bgx = uPrev(10,1);
bgy = uPrev(11,1);
bgz = uPrev(12,1);

bax = uPrev(13,1);
bay = uPrev(14,1);
baz = uPrev(15,1);

x = uPrev(4,1);
y = uPrev(5,1);
z = uPrev(6,1);

x_dot = [p_dot_x; 
         p_dot_y; 
         p_dot_z; 
         (bgy - omega_m_y)*((cos(z)*(cos(x)*sin(z) - cos(z)*sin(x)*sin(y)))/cos(y) - (sin(z)*(cos(x)*cos(z) + sin(x)*sin(y)*sin(z)))/cos(y)) - (bgz - omega_m_z)*((cos(z)*(sin(x)*sin(z) + cos(x)*cos(z)*sin(y)))/cos(y) - (sin(z)*(cos(z)*sin(x) - cos(x)*sin(y)*sin(z)))/cos(y)) - (bgx - omega_m_x)*(cos(z)^2 + sin(z)^2); 
         (cos(z)*(cos(z)*sin(x) - cos(x)*sin(y)*sin(z)) + sin(z)*(sin(x)*sin(z) + cos(x)*cos(z)*sin(y)))*(bgz - omega_m_z) - (cos(z)*(cos(x)*cos(z) + sin(x)*sin(y)*sin(z)) + sin(z)*(cos(x)*sin(z) - cos(z)*sin(x)*sin(y)))*(bgy - omega_m_y); 
         -(bgz - omega_m_z)*(cos(x)*cos(y) + (cos(z)*sin(y)*(sin(x)*sin(z) + cos(x)*cos(z)*sin(y)))/cos(y) - (sin(y)*sin(z)*(cos(z)*sin(x) - cos(x)*sin(y)*sin(z)))/cos(y)) - (bgy - omega_m_y)*(cos(y)*sin(x) - (cos(z)*sin(y)*(cos(x)*sin(z) - cos(z)*sin(x)*sin(y)))/cos(y) + (sin(y)*sin(z)*(cos(x)*cos(z) + sin(x)*sin(y)*sin(z)))/cos(y)) - (bgx - omega_m_x)*(sin(y)*cos(z)^2 + sin(y)*sin(z)^2 - sin(y)); 
         (acc_m_z - baz)*(sin(x)*sin(z) + cos(x)*cos(z)*sin(y)) - (acc_m_y - bay)*(cos(x)*sin(z) - cos(z)*sin(x)*sin(y)) + cos(y)*cos(z)*(acc_m_x - bax); 
         (acc_m_y - bay)*(cos(x)*cos(z) + sin(x)*sin(y)*sin(z)) - (acc_m_z - baz)*(cos(z)*sin(x) - cos(x)*sin(y)*sin(z)) + cos(y)*sin(z)*(acc_m_x - bax); 
         cos(x)*cos(y)*(acc_m_z - baz) - sin(y)*(acc_m_x - bax) + cos(y)*sin(x)*(acc_m_y - bay) - (49/5); 
         (0); 
         (0); 
         (0); 
         (0); 
         (0); 
         (0)];

A_t = [(0), (0), (0), (0), (0), (0), (1), (0), (0), (0), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (1), (0), (0), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (1), (0), (0), (0), (0), (0), (0); (0), (0), (0), - (bgy - omega_m_y)*((cos(z)*(sin(x)*sin(z) + cos(x)*cos(z)*sin(y)))/cos(y) - (sin(z)*(cos(z)*sin(x) - cos(x)*sin(y)*sin(z)))/cos(y)) - (bgz - omega_m_z)*((cos(z)*(cos(x)*sin(z) - cos(z)*sin(x)*sin(y)))/cos(y) - (sin(z)*(cos(x)*cos(z) + sin(x)*sin(y)*sin(z)))/cos(y)), - (bgz - omega_m_z)*(cos(x)*cos(z)^2 + cos(x)*sin(z)^2 + (cos(z)*sin(y)*(sin(x)*sin(z) + cos(x)*cos(z)*sin(y)))/cos(y)^2 - (sin(y)*sin(z)*(cos(z)*sin(x) - cos(x)*sin(y)*sin(z)))/cos(y)^2) - (bgy - omega_m_y)*(cos(z)^2*sin(x) + sin(x)*sin(z)^2 - (cos(z)*sin(y)*(cos(x)*sin(z) - cos(z)*sin(x)*sin(y)))/cos(y)^2 + (sin(y)*sin(z)*(cos(x)*cos(z) + sin(x)*sin(y)*sin(z)))/cos(y)^2), (0), (0), (0), (0), - cos(z)^2 - sin(z)^2, ((cos(z)*(cos(x)*sin(z) - cos(z)*sin(x)*sin(y)))/cos(y) - (sin(z)*(cos(x)*cos(z) + sin(x)*sin(y)*sin(z)))/cos(y)), (sin(z)*(cos(z)*sin(x) - cos(x)*sin(y)*sin(z)))/cos(y) - (cos(z)*(sin(x)*sin(z) + cos(x)*cos(z)*sin(y)))/cos(y), (0), (0), (0); (0), (0), (0), (cos(z)*(cos(z)*sin(x) - cos(x)*sin(y)*sin(z)) + sin(z)*(sin(x)*sin(z) + cos(x)*cos(z)*sin(y)))*(bgy - omega_m_y) + (cos(z)*(cos(x)*cos(z) + sin(x)*sin(y)*sin(z)) + sin(z)*(cos(x)*sin(z) - cos(z)*sin(x)*sin(y)))*(bgz - omega_m_z), (0), (0), (0), (0), (0), (0), - cos(z)*(cos(x)*cos(z) + sin(x)*sin(y)*sin(z)) - sin(z)*(cos(x)*sin(z) - cos(z)*sin(x)*sin(y)), (cos(z)*(cos(z)*sin(x) - cos(x)*sin(y)*sin(z)) + sin(z)*(sin(x)*sin(z) + cos(x)*cos(z)*sin(y))), (0), (0), (0); (0), (0), (0), (bgz - omega_m_z)*(cos(y)*sin(x) - (cos(z)*sin(y)*(cos(x)*sin(z) - cos(z)*sin(x)*sin(y)))/cos(y) + (sin(y)*sin(z)*(cos(x)*cos(z) + sin(x)*sin(y)*sin(z)))/cos(y)) - (bgy - omega_m_y)*(cos(x)*cos(y) + (cos(z)*sin(y)*(sin(x)*sin(z) + cos(x)*cos(z)*sin(y)))/cos(y) - (sin(y)*sin(z)*(cos(z)*sin(x) - cos(x)*sin(y)*sin(z)))/cos(y)), - (bgy - omega_m_y)*(sin(z)*(cos(x)*cos(z) + sin(x)*sin(y)*sin(z)) - cos(z)*(cos(x)*sin(z) - cos(z)*sin(x)*sin(y)) - sin(x)*sin(y) + cos(z)^2*sin(x)*sin(y) + sin(x)*sin(y)*sin(z)^2 - (cos(z)*sin(y)^2*(cos(x)*sin(z) - cos(z)*sin(x)*sin(y)))/cos(y)^2 + (sin(y)^2*sin(z)*(cos(x)*cos(z) + sin(x)*sin(y)*sin(z)))/cos(y)^2) - (bgx - omega_m_x)*(cos(y)*cos(z)^2 + cos(y)*sin(z)^2 - cos(y)) - (bgz - omega_m_z)*(cos(z)*(sin(x)*sin(z) + cos(x)*cos(z)*sin(y)) - cos(x)*sin(y) - sin(z)*(cos(z)*sin(x) - cos(x)*sin(y)*sin(z)) + cos(x)*cos(z)^2*sin(y) + cos(x)*sin(y)*sin(z)^2 + (cos(z)*sin(y)^2*(sin(x)*sin(z) + cos(x)*cos(z)*sin(y)))/cos(y)^2 - (sin(y)^2*sin(z)*(cos(z)*sin(x) - cos(x)*sin(y)*sin(z)))/cos(y)^2), (0), (0), (0), (0), - sin(y)*cos(z)^2 - sin(y)*sin(z)^2 + sin(y), (cos(z)*sin(y)*(cos(x)*sin(z) - cos(z)*sin(x)*sin(y)))/cos(y) - cos(y)*sin(x) - (sin(y)*sin(z)*(cos(x)*cos(z) + sin(x)*sin(y)*sin(z)))/cos(y), (sin(y)*sin(z)*(cos(z)*sin(x) - cos(x)*sin(y)*sin(z)))/cos(y) - (cos(z)*sin(y)*(sin(x)*sin(z) + cos(x)*cos(z)*sin(y)))/cos(y) - cos(x)*cos(y), (0), (0), (0); (0), (0), (0), (acc_m_y - bay)*(sin(x)*sin(z) + cos(x)*cos(z)*sin(y)) + (acc_m_z - baz)*(cos(x)*sin(z) - cos(z)*sin(x)*sin(y)), cos(x)*cos(y)*cos(z)*(acc_m_z - baz) - cos(z)*sin(y)*(acc_m_x - bax) + cos(y)*cos(z)*sin(x)*(acc_m_y - bay), (acc_m_z - baz)*(cos(z)*sin(x) - cos(x)*sin(y)*sin(z)) - (acc_m_y - bay)*(cos(x)*cos(z) + sin(x)*sin(y)*sin(z)) - cos(y)*sin(z)*(acc_m_x - bax), (0), (0), (0), (0), (0), (0), -cos(y)*cos(z), (cos(x)*sin(z) - cos(z)*sin(x)*sin(y)), - sin(x)*sin(z) - cos(x)*cos(z)*sin(y); (0), (0), (0), - (acc_m_y - bay)*(cos(z)*sin(x) - cos(x)*sin(y)*sin(z)) - (acc_m_z - baz)*(cos(x)*cos(z) + sin(x)*sin(y)*sin(z)), cos(x)*cos(y)*sin(z)*(acc_m_z - baz) - sin(y)*sin(z)*(acc_m_x - bax) + cos(y)*sin(x)*sin(z)*(acc_m_y - bay), (acc_m_z - baz)*(sin(x)*sin(z) + cos(x)*cos(z)*sin(y)) - (acc_m_y - bay)*(cos(x)*sin(z) - cos(z)*sin(x)*sin(y)) + cos(y)*cos(z)*(acc_m_x - bax), (0), (0), (0), (0), (0), (0), -cos(y)*sin(z), - cos(x)*cos(z) - sin(x)*sin(y)*sin(z), (cos(z)*sin(x) - cos(x)*sin(y)*sin(z)); (0), (0), (0), cos(x)*cos(y)*(acc_m_y - bay) - cos(y)*sin(x)*(acc_m_z - baz), - cos(y)*(acc_m_x - bax) - cos(x)*sin(y)*(acc_m_z - baz) - sin(x)*sin(y)*(acc_m_y - bay), (0), (0), (0), (0), (0), (0), (0), sin(y), -cos(y)*sin(x), -cos(x)*cos(y); (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0)];
U_t = [(0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0); - cos(z)^2 - sin(z)^2, (cos(z)*(cos(x)*sin(z) - cos(z)*sin(x)*sin(y)))/cos(y) - (sin(z)*(cos(x)*cos(z) + sin(x)*sin(y)*sin(z)))/cos(y), (sin(z)*(cos(z)*sin(x) - cos(x)*sin(y)*sin(z)))/cos(y) - (cos(z)*(sin(x)*sin(z) + cos(x)*cos(z)*sin(y)))/cos(y), (0), (0), (0), (0), (0), (0), (0), (0), (0); (0), - cos(z)*(cos(x)*cos(z) + sin(x)*sin(y)*sin(z)) - sin(z)*(cos(x)*sin(z) - cos(z)*sin(x)*sin(y)), cos(z)*(cos(z)*sin(x) - cos(x)*sin(y)*sin(z)) + sin(z)*(sin(x)*sin(z) + cos(x)*cos(z)*sin(y)), (0), (0), (0), (0), (0), (0), (0), (0), (0); - sin(y)*cos(z)^2 - sin(y)*sin(z)^2 + sin(y), (cos(z)*sin(y)*(cos(x)*sin(z) - cos(z)*sin(x)*sin(y)))/cos(y) - cos(y)*sin(x) - (sin(y)*sin(z)*(cos(x)*cos(z) + sin(x)*sin(y)*sin(z)))/cos(y), (sin(y)*sin(z)*(cos(z)*sin(x) - cos(x)*sin(y)*sin(z)))/cos(y) - (cos(z)*sin(y)*(sin(x)*sin(z) + cos(x)*cos(z)*sin(y)))/cos(y) - cos(x)*cos(y), (0), (0), (0), (0), (0), (0), (0), (0), (0); (0), (0), (0), -cos(y)*cos(z), (cos(x)*sin(z) - cos(z)*sin(x)*sin(y)), - sin(x)*sin(z) - cos(x)*cos(z)*sin(y), (0), (0), (0), (0), (0), (0); (0), (0), (0), -cos(y)*sin(z), - cos(x)*cos(z) - sin(x)*sin(y)*sin(z), (cos(z)*sin(x) - cos(x)*sin(y)*sin(z)), (0), (0), (0), (0), (0), (0); (0), (0), (0), sin(y), -cos(y)*sin(x), -cos(x)*cos(y), (0), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (1), (0), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (1), (0), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (1), (0), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (0), (1), (0), (0); (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (1), (0); (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (0), (1)];

Ft = eye(15) + dt*A_t;
Vt = U_t;
Qd = eye(12)*dt;

uEst = uPrev + dt*x_dot;
covarEst = Ft*covarPrev*Ft' + Vt*Qd*Vt';

end

