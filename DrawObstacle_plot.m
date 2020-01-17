%% 绘制所有障碍物位置

% 输入参数：obstacle 所有障碍物的坐标   obstacleR 障碍物的半径
function [] = DrawObstacle_plot(obstacle,obstacleR)
r = obstacleR; 
obstacle=obstacle';
theta = 0:pi/20:2*pi;
for id=1:length(obstacle(:,1))
 x = r * cos(theta) + obstacle(id,1); 
 y = r * sin(theta) + obstacle(id,2);
 plot(x,y,'-m');hold on; 
end
% plot(obstacle(:,1),obstacle(:,2),'*m');hold on;              % 绘制所有障碍物位置