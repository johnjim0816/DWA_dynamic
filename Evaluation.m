%% 评价函数 内部负责产生可用轨迹
% 输入参数 ：当前状态、参数允许范围（窗口）、目标点、障碍物位置、障碍物半径、评价函数的参数
% 返回参数：evalDB N*5  每行一组可用参数 分别为 速度、角速度、航向得分、距离得分、速度得分
%         trajDB      每5行一条轨迹 每条轨迹包含 前向预测时间/dt + 1 = 31 个轨迹点（见生成轨迹函数）
function [evalDB,trajDB] = Evaluation(x,Vr,goal,ob,R,model,evalParam) 
evalDB = [];
trajDB = [];
for vt = Vr(1):model(5):Vr(2)       %根据速度分辨率遍历所有可用速度： 最小速度和最大速度 之间 速度分辨率 递增 
    for ot=Vr(3):model(6):Vr(4)     %根据角度分辨率遍历所有可用角速度： 最小角速度和最大角速度 之间 角度分辨率 递增  
        % 轨迹推测; 得到 xt: 机器人向前运动后的预测位姿; traj: 当前时刻 到 预测时刻之间的轨迹（由轨迹点组成）
        [xt,traj] = GenerateTrajectory(x,vt,ot,evalParam(4),model);  %evalParam(4),前向模拟时间;
        % 各评价函数的计算
        heading = CalcHeadingEval(xt,goal); % 前项预测终点的航向得分  偏差越小分数越高
        dist    = CalcDistEval(xt,ob,R);    % 前项预测终点 距离最近障碍物的间隙得分 距离越远分数越高
        vel     = abs(vt);                  % 速度得分 速度越快分越高
        stopDist = CalcBreakingDist(vel,model); % 制动距离的计算
        if dist > stopDist % 如果可能撞到最近的障碍物 则舍弃此路径 （到最近障碍物的距离 大于 刹车距离 才取用）
            evalDB = [evalDB;[vt ot heading dist vel]];
            trajDB = [trajDB;traj];   % 每5行 一条轨迹  
        end
    end
end
