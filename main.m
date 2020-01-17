function [] = main()

close all;
clear all;

disp('Dynamic Window Approach program with dynamic obstacles start!!')

%% 机器人的状态[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]

x = [0 0 pi/10 0 0]'; % 赋值初始状态: 5x1矩阵 列矩阵 位置(0,0) 航向pi/10 ,速度、角速度均为0

% 下标宏定义 状态[x(m),y(m),yaw(Rad),v(m/s),w(rad/s)]
POSE_X      = 1;  %坐标 X
POSE_Y      = 2;  %坐标 Y
YAW_ANGLE   = 3;  %机器人航向角
V_SPD       = 4;  %机器人速度
W_ANGLE_SPD = 5;  %机器人角速度 

goal = [10,10];   % 目标点位置 [x(m),y(m)]

%% 障碍物状态列表 [x(m) y(m) yaw(Rad) v(m/s) w(rad/s)]
%  默认障碍物以恒定速度运行，即yaw,v,w=0

% obstacle=[0 2 0     0.2 0 ;
%           2 4 pi/10 0.1 0 ;
%           2 5 pi/2  0.1 0 ;      
%           4 2 pi/2  0.1 0 ;
%           5 4 pi/2  0.1 0 ;
%           5 6 pi/2  0.1 0 ;
%           5 9 pi/2  0.1 0 ;
%           8 8 pi/2  0.1 0 ;
%           8 9 pi/2  0.1 0 ;
%           7 9 1.5*pi  0.1 0 ;
%           ]';

%% 设置随机障碍状态信息
% 障碍数量
number_of_obstacle = 20;
% 为了使得障碍物x，y坐标在1-9之间(包括小数)，yaw在0-2*pi[rad]之间，v在0-0.5[m/s],w=0
% rad/s,设置系数A_ob和B_ob如下
A_ob=[8 0    0   0 0;
      0 8    0   0 0;
      0 0 2*pi   0 0;
      0 0    0 0.5 0;
      0 0    0   0 0];

B_ob=[1 1 0 0 0];
B_ob=repmat(B_ob,number_of_obstacle,1);

% 生成障碍状态参数列表
obstacle=rand(number_of_obstacle,5)*A_ob+B_ob;
obstacle=obstacle';
            
obstacleR = 0.5;% 冲突判定用的障碍物半径

global dt; 
dt = 0.1;% 时间[s]

% 机器人运动学模型参数
% 最高速度[m/s],最高旋转速度[rad/s],加速度[m/ss],旋转加速度[rad/ss],速度分辨率[m/s],转速分辨率[rad/s]]
model = [1.0,toRadian(20.0),0.2,toRadian(50.0),0.01,toRadian(1)];

%定义model的下标
MD_MAX_V    = 1;%   最高速度m/s]
MD_MAX_W    = 2;%   最高旋转速度[rad/s]
MD_ACC      = 3;%   加速度[m/ss]
MD_VW       = 4;%   旋转加速度[rad/ss]
MD_V_RESOLUTION  = 5;%  速度分辨率[m/s]
MD_W_RESOLUTION  = 6;%  转速分辨率[rad/s]]


% 评价函数参数 [heading,dist,velocity,predictDT]
% 航向得分的比重、距离得分的比重、速度得分的比重、向前模拟轨迹的时间
evalParam = [0.05, 0.2 ,0.1, 3.0];

area      = [-1 11 -1 11];% 模拟区域范围 [xmin xmax ymin ymax]

% 模拟实验的结果
result.x=[];   %累积存储走过的轨迹点的状态值 
tic; % 估算程序运行时间开始

writerObj=VideoWriter('./results/test.avi');  % 定义一个视频文件用来存动画
open(writerObj);                    % 打开该视频文件

%% Main loop   循环运行 5000次 指导达到目的地 或者 5000次运行结束
for i = 1:5000  
    % DWA参数输入 返回控制量 u = [v(m/s),w(rad/s)] 和 轨迹
    [u,traj] = DynamicWindowApproach(x,model,goal,evalParam,obstacle,obstacleR);
    
    x = f(x,u);% 机器人移动到下一个时刻的状态量 根据当前速度和角速度推导 下一刻的位置和角度
    obstacle=MoveObstacle(obstacle);
    % 历史轨迹的保存
    result.x = [result.x; x'];  %最新结果 以列的形式 添加到result.x
    
    % 是否到达目的地
    if norm(x(POSE_X:POSE_Y)-goal')<0.5   % norm函数来求得坐标上的两个点之间的距离
        disp('Arrive Goal!!');break;
    end
    
    %====Animation====
    hold off;               % 关闭图形保持功能。 新图出现时，取消原图的显示。
    ArrowLength = 0.5;      % 箭头长度
    
    % 机器人
    % quiver(x,y,u,v) 在 x 和 y 中每个对应元素对组所指定的坐标处将向量绘制为箭头
    quiver(x(POSE_X), x(POSE_Y), ArrowLength*cos(x(YAW_ANGLE)), ArrowLength*sin(x(YAW_ANGLE)), 'ok'); % 绘制机器人当前位置的航向箭头
    hold on;                                                     %启动图形保持功能，当前坐标轴和图形都将保持，从此绘制的图形都将添加在这个图形的基础上，并自动调整坐标轴的范围
    
    plot(result.x(:,POSE_X),result.x(:,POSE_Y),'-b');hold on;    % 绘制走过的所有位置 所有历史数据的 X、Y坐标
    plot(goal(1),goal(2),'*r');hold on;                          % 绘制目标位置
    
    %plot(obstacle(:,1),obstacle(:,2),'*k');hold on;              % 绘制所有障碍物位置
    DrawObstacle_plot(obstacle,obstacleR);
    
    % 探索轨迹 画出待评价的轨迹
    if ~isempty(traj) %轨迹非空
        for it=1:length(traj(:,1))/5    %计算所有轨迹数  traj 每5行数据 表示一条轨迹点
            ind = 1+(it-1)*5; %第 it 条轨迹对应在traj中的下标 
            plot(traj(ind,:),traj(ind+1,:),'-g');hold on;  %根据一条轨迹的点串画出轨迹   traj(ind,:) 表示第ind条轨迹的所有x坐标值  traj(ind+1,:)表示第ind条轨迹的所有y坐标值
        end
    end
    
    axis(area); %根据area设置当前图形的坐标范围，分别为x轴的最小、最大值，y轴的最小最大值
    grid on;
    drawnow;  %刷新屏幕. 当代码执行时间长，需要反复执行plot时，Matlab程序不会马上把图像画到figure上，这时，要想实时看到图像的每一步变化情况，需要使用这个语句。
    frame = getframe;            %// 把图像存入视频文件中
    writeVideo(writerObj,frame); %// 将帧写入视频
end

close(writerObj); %// 关闭视频文件句柄

toc  %输出程序运行时间  形式：时间已过 ** 秒。





