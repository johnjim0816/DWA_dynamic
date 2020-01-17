%% 计算制动距离 
%根据运动学模型计算制动距离, 也可以考虑成走一段段圆弧的累积 简化可以当一段段小直线的累积
function stopDist = CalcBreakingDist(vel,model)
global dt;
MD_ACC   = 3;% 
stopDist=0;
while vel>0   %给定加速度的条件下 速度减到0所走的距离
    stopDist = stopDist + vel*dt;% 制动距离的计算 
    vel = vel - model(MD_ACC)*dt;% 
end

