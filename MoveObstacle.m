%% 更新动态障碍物信息
%% update info of dynami obstacles
% 输入输出: obstacle矩阵(5*n，其中n是obstacle个数)，每个列向量分别为 [x(m) y(m) yaw(Rad) v(m/s) w(rad/s)]
% 对每个障碍物，更新公式应为：x=x+v*dt*cos(yaw),y=y+v*dt*sin(yaw),yaw=yaw+w*dt,v=v,w=w
function obstacle=MoveObstacle(obstacle)
global dt;
B = [0 0 0 dt 0
     0 0 0 dt 0
     0 0 0 0  dt
     0 0 0 0  0
     0 0 0 0  0];
delta= B* obstacle;
for num=1:length(obstacle(1,:))
    temp = [cos(obstacle(3,num)) 0 0 0 0
            0  sin(obstacle(3,num)) 0 0 0
            0 0 1 0 0
            0 0 0 0 0
            0 0 0 0 0];
     delta(:,num)=temp*delta(:,num);
end
obstacle=obstacle+delta;
