function Fcmax = Squat(height, weight, wbar, highlow, stance, yfoot, ak)
% 杠铃深蹲模拟程序
% 作者：宁波大学，周吕文，zhoulvwen@nbu.edu.cn
% 2024/04/29

if nargin==0
    height  = 180; weight = 75;      % 身高[cm] & 体重[kg]
    stance  = 20;  yfoot  = 6;       % 站姿角[deg] & 脚的侧向偏移距离[cm]
    highlow = 0;   wbar   = 50;      % 高低杠[cm ] & 杠铃重量[kg]
    ak   = 60;                       % 踝关节最终屈曲[deg]
    isplot = true;
else
    isplot = false;
end

% 身体重部分长度、重量和重心高度比例
[trunk, thigh, shank, upperArm, foreArm, hand, headNeck, ...
         foot, neck, head, hip, shoulder, arm] = sportsman(height, weight);

hand.yshift = 30;                           % [cm ] 杠铃上肩到手的距离
foot.yshift = yfoot;                        % [cm ] 两脚侧向偏移距离

bar.wt = wbar;                              % [kg ] 杠铃重
bar.R = 22.5;                               % [cm ] 杠铃半径
bar.L = 110;                                % [cm ] 杠铃半长

n = 101; % 模拟步数

[ankle, toe, heel, shank, foot, thigh, hand, shoulder] = initpos(stance,...
    bar, trunk, thigh, shank, arm, hand, headNeck, foot, hip, shoulder);

% 最终状态下 踝关节屈曲（小腿与脚的夹角）和 大腿在xz平面投影与竖直方向的夹角
ankle.a(n) = ak;
thigh.a(n) = 90;

for i = 1:n
    ankle.a(i) = ankle.a(1) + (ankle.a(n) - ankle.a(1)) * (i-1)/(n-1);
    thigh.a(i) = thigh.a(1) + (thigh.a(n) - thigh.a(1)) * (i-1)/(n-1);

    % 小腿方向 v = 脚方向绕 ax 旋转 ankle.a(i) 度，并据此求解膝关节坐标
    v = vRotAxis(foot.v, -ankle.a(i), shank.axis);
    knee.x(i) =  shank.L*v(1) + ankle.x;    % 膝的水向坐标（向前）
    knee.y(i) =  shank.L*v(2) + ankle.y;    % 膝的水向坐标（侧前）
    knee.z(i) =  shank.L*v(3) + ankle.z;    % 膝的竖直坐标
 
    % x 分量 = sqrt( (Thigh.L 在 x-y 平面的投影)^2 - (y 分量)^2 )
    thighxz = sqrt(thigh.L^2 - (hip.W-knee.y(i))^2);
    hip.x(i) = knee.x(i) - thighxz*sind(thigh.a(i));
    hip.z(i) = knee.z(i) + thighxz*cosd(thigh.a(i));

    % 二分法求杠铃最佳位置：质心 x 坐标为 0
    xc = -inf;
    xmin = -20;
    xmax = 50;
    while abs(real(xc))>1e-3 
        shoulder.x(i) = (xmin + xmax)/2;
        trunk.a(i) = asind( (shoulder.x(i)-hip.x(i))/trunk.L);
        shoulder.z(i) = hip.z(i) + trunk.L*cosd(trunk.a(i));
        
        hand.x(i) = shoulder.x(i) - highlow * sind(trunk.a(i));
        hand.z(i) = shoulder.z(i) - highlow * cosd(trunk.a(i));
        
        bar.x(i) = hand.x(i); bar.z(i) = hand.z(i);

        [dy, db] = circcirc(0, 0, upperArm.L, ...
                           hand.y-shoulder.y, highlow, foreArm.L+hand.L);
        dy = dy(db>highlow);
        db = db(db>highlow);
        elbow.y(i) = shoulder.y + dy;
        elbow.x(i) = shoulder.x(i) - db*sind(trunk.a(i));
        elbow.z(i) = shoulder.z(i) - db*cosd(trunk.a(i));

        xc = centroidx(weight, bar, headNeck, trunk, thigh, shank, ...
            upperArm, shoulder, foreArm, hand, hip, knee, ankle, elbow, i);

        if real(xc)<0; xmin = shoulder.x(i); else xmax = shoulder.x(i); end
    end
end

% 绘制动画
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