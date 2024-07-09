function Fcmax = Deadlift(height, weight, wbar, stance, yfoot, yhand, ak)
% 杠铃硬拉模拟程序
% 作者：宁波大学，周吕文，zhoulvwen@nbu.edu.cn
% 2024/04/29

if nargin==0
    height  = 180; weight = 75;      % 身高[cm] & 体重[kg]
    stance  = 20;  yfoot  = 6;       % 站姿角[deg] & 脚的侧向偏移距离[cm]
    yhand   = 0;   wbar   = 50;      % 手侧向偏移[cm] & 杠铃重量[kg]
    ak      = 65;                    % 踝关节最终屈曲[deg]
    isplot = true;
else
    isplot = false;
end

% 身体重部分长度、重量和重心高度比例
[trunk, thigh, shank, upperArm, foreArm, hand, headNeck, ...
       foot, neck, head, hip, shoulder, arm] = sportsman(height, weight);

hand.yshift = yhand;                        % [cm ] 杠铃上肩到手的距离
foot.yshift = yfoot;                        % [cm ] 两脚侧向偏移距离

bar.wt = wbar;                              % [kg ] 杠铃重
bar.R = 22.5;                               % [cm ] 杠铃半径
bar.L = 110;                                % [cm ] 杠铃半长 

[ankle, toe, heel, shank, foot, thigh, hand, shoulder] = initpos(stance,...
    bar, trunk, thigh, shank, arm, hand, headNeck, foot, hip, shoulder);

n = 101;

% 最终状态下 踝关节屈曲（小腿与脚的夹角）和 杠玲位置
am = akmin(ankle, bar, trunk, thigh, shank, arm, hand, headNeck, foot, hip);
ankle.a(n) = max(ak, am);

shoulder.z(n) = sqrt(arm.L^2 - hand.yshift^2) + bar.R; 

for i = 1:n
    % 踝关节屈曲（小腿与竖直方向的夹角）
    ankle.a(i) = ankle.a(1) + (ankle.a(n) - ankle.a(1)) * (i-1)/(n-1);
    shoulder.z(i) = shoulder.z(1) + (shoulder.z(n)-shoulder.z(1))*(i-1)/(n-1);

    % 小腿方向 v = 脚方向绕 ax 旋转 ankle.a(i) 度，并据此求解膝关节坐标
    v = vRotAxis(foot.v, -ankle.a(i), shank.axis);
    knee.x(i) =  shank.L*v(1) + ankle.x;    % 膝的水向坐标（向前）
    knee.y(i) =  shank.L*v(2) + ankle.y;    % 膝的水向坐标（侧前）
    knee.z(i) =  shank.L*v(3) + ankle.z;    % 膝的竖直坐标

    % 二分法求杠铃最佳位置：质心 x 坐标为 0
    xc = -inf;
    xmin = -20;
    xmax = 50;

    while abs(xc)>1e-3
        shoulder.x(i) = (xmin + xmax)/2;

        % 手（杠玲圆盘中心）竖直的位置
        hand.z(i) = shoulder.z(i) - sqrt(arm.L^2 - hand.yshift^2);
        hand.x(i) = shoulder.x(i);

        bar.x(i) = hand.x(i); bar.z(i) = hand.z(i);
        
        % 肘关节位置
        upap = upperArm.L/arm.L; % 上臂占整个手臂的比例
        elbow.x(i) = shoulder.x(i) + upap*(hand.x(i)-shoulder.x(i));
        elbow.y(i) = shoulder.y    + upap*(hand.y   -shoulder.y   );
        elbow.z(i) = shoulder.z(i) + upap*(hand.z(i)-shoulder.z(i));

        % 大腿在 x-z 平面内的投影长度
        femurxz = sqrt(thigh.L^2 - (hip.W-knee.y(i))^2 );

        % 在 x-y 平面，以膝为心大腿为半径的圆与以肩为心躯干为半径的圆的交点即为 hip
        [hx, hz] = circcirc(    knee.x(i),     knee.z(i), femurxz, ...
                            shoulder.x(i), shoulder.z(i), trunk.L);
        if isnan(hx)
            fftp = femurxz/(femurxz+trunk.L);
            hip.x(i) = knee.x(i) + fftp*(shoulder.x(i)-knee.x(i));
            hip.z(i) = knee.z(i) + fftp*(shoulder.z(i)-knee.z(i));
        else
            [hip.x(i), jmin] = min(hx);
            hip.z(i) = hz(jmin);
        end

        % 背与竖直方向夹角
        trunk.a(i) = asind( (shoulder.x(i)-hip.x(i))/trunk.L); 

        xc = centroidx(weight, bar, headNeck, trunk, thigh, shank, ...
            upperArm, shoulder, foreArm, hand, hip, knee, ankle, elbow, i);

        if xc<0; xmin = shoulder.x(i); else; xmax = shoulder.x(i); end
    end
end

% % 绘制动画
if isplot
    plotpos(neck,trunk,ankle,toe,heel,knee,hip,shoulder,elbow,hand,head,bar)
end

hip.a  = jointAngle(shoulder, hip, knee, 'xz');
knee.a = jointAngle(hip, knee, ankle, 'xyz');


% 三个关键关节的力矩
Mnt = moments(bar, headNeck, trunk, thigh, shank, upperArm, shoulder, ...
                           foreArm, hand, hip, knee, ankle, elbow, isplot);

% L5/S1 受到的压力 Fc 和剪力 Fs，及背棘肌张力 Fm
Mhip = Mnt('hip'); Mhip = sum(Mhip{:});
m = headNeck.wt+trunk.wt+arm.wt+bar.wt;
K = jointAngle(hip, knee, ankle, 'xz');
[Fc, Fs, Fm] = L5S1Force(hip.a, trunk.a, K, Mhip, m, shoulder, isplot);

Fcmax = max(Fc);

%% ------------------------------------------------------------------------

function ak = akmin(ankle,bar,trunk,thigh,shank,arm,hand,headNeck,foot,hip)
% 求 ankle.a 的最小值：最后状态腿刚好碰到杠玲

amin = 0; amax = 90;
Moment = inf;

while abs(Moment)>1
    ankle.a = (amin+amax)/2;

    shoulder.z = sqrt(arm.L^2 - hand.yshift^2) + bar.R;
    v = vRotAxis(foot.v, -ankle.a, shank.axis);
    knee.x =  shank.L*v(1) + ankle.x;    % 膝的水向坐标（向前）
    knee.y =  shank.L*v(2) + ankle.y;    % 膝的水向坐标（侧前）
    knee.z =  shank.L*v(3) + ankle.z;    % 膝的竖直坐标

    shoulder.x = ankle.x+(bar.R - ankle.z)*v(1)/v(3);
    hand.z = shoulder.z - sqrt(arm.L^2 - hand.yshift^2);
    hand.x = shoulder.x;

    % 大腿在 x-z 平面内的投影长度
    femurxz = sqrt(thigh.L^2 - (hip.W-knee.y)^2 );

    % 在 x-y 平面，以膝为心大腿为半径的圆与以肩为心躯干为半径的圆的交点即为 hip
    [hx, hz] = circcirc(knee.x, knee.z, femurxz, shoulder.x, shoulder.z, trunk.L);
    hip.x = min(hx);
    
    % 背与竖直方向夹角
    trunk.a = asind( (shoulder.x-hip.x)/trunk.L );

    headNeckLx = cosd(trunk.a)*headNeck.L; % 头颈长度在 x 方向上的投影

    % 对(0,0) 的矩
    Moment = hand.x * (bar.wt+arm.wt) + ...
        (shoulder.x + headNeck.CoG*headNeckLx         ) * headNeck.wt + ...
        (     hip.x +    trunk.CoG*(shoulder.x- hip.x)) * trunk.wt + ...
        (     hip.x +    thigh.CoG*(    knee.x- hip.x)) * thigh.wt + ...
        (    knee.x +    shank.CoG*(   ankle.x-knee.x)) * shank.wt;
    if Moment>0; amin = ankle.a; else; amax = ankle.a; end
end
ak = amax;