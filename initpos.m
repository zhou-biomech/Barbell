function [ankle, toe, heel, shank, foot, thigh, hand, shoulder] = initpos(...
stance, bar, trunk, thigh, shank, arm, hand, headNeck, foot, hip, shoulder, xsb)

if nargin==11; xsb = 0; end

ankle.x = -1/4*foot.L*cosd(stance);
ankle.y = hip.W+foot.yshift;
ankle.z = 0.16*shank.L;

toe.x = 3/4*foot.L*cosd(stance) + ankle.x;  % 脚尖到踝关节距离为 3/4 个脚
toe.y = 3/4*foot.L*sind(stance) + ankle.y;
toe.z = 0.2;

heel.x = -1/4*foot.L*cosd(stance) + ankle.x;% 脚跟到踝关节距离为 1/4 个脚
heel.y = -1/4*foot.L*sind(stance) + ankle.y;
heel.z = 0.2;

% 身体直立，两腿分开时，腿与地面夹角
as = asind(foot.yshift/(shank.L+thigh.L));

% 估计初始状态身体前倾角度（确保初始状态重心落在脚中心点附近）
% ab = asind(foot.L/4*cosd(stance)/((shank.L+thigh.L)*cosd(as)+trunk.L));
m = shank.wt+thigh.wt+trunk.wt+headNeck.wt+bar.wt+arm.wt;
legxz = (shank.L+thigh.L)*cosd(as);
ab = asind( (1/4*foot.L*cosd(stance)*m -xsb*bar.wt) /( ...
    (shank.L*(1-shank.CoG)        )*cosd(as)*shank.wt+ ...
    (shank.L+(1-thigh.CoG)*thigh.L)*cosd(as)*thigh.wt + ...
    (legxz  +(1-trunk.CoG)*trunk.L)*trunk.wt + ...
    (legxz+trunk.L+headNeck.CoG*headNeck.L)*headNeck.wt + ...
    (legxz+trunk.L)*(bar.wt+arm.wt) ));

% 小腿初始方向
shank.v = [cosd(as)*sind(ab), -sind(as), cosd(as)*cosd(ab)];

% 脚朝向和大腿方向所在平面的法向量，之后小腿将绕此轴转动
foot.v = [cosd(stance), sind(stance), 0];
shank.axis = cross(shank.v, foot.v);

% 手的 y 位置 = 肩的位置 + y方向偏移
hand.y = shoulder.y + hand.yshift;

% 踝关节初始屈曲（小腿与脚的夹角）
ankle.a(1) = acosd(dot(foot.v, shank.v));

% 肩膀初始高度
shoulder.z(1) = (legxz+trunk.L)*cosd(ab)  + ankle.z;

% 大腿在xz方向的分量与竖直的初始终夹角
thigh.a(1) = -atand(shank.v(1)/shank.v(3));