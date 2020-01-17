%% Motion Model 根据当前状态推算下一个控制周期（dt）的状态
%
% u = [vt; wt];当前时刻的速度、角速度 x = 状态[x(m) y(m) yaw(Rad) v(m/s) w(rad/s)]'(5*1矩阵)
function x = f(x, u)
global dt;
F = [1 0 0 0 0
     0 1 0 0 0
     0 0 1 0 0
     0 0 0 0 0
     0 0 0 0 0];
 
B = [dt*cos(x(3)) 0
    dt*sin(x(3)) 0
    0 dt
    1 0
    0 1];
 
x= F*x+B*u; 