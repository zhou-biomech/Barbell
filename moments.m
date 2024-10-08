function Mnt = moments(bar, headNeck, trunk, thigh, shank, upperArm, ...
                  shoulder, foreArm, hand, hip, knee, ankle, elbow, isplot)
if nargin==13; isplot = true; end

g = 9.8/100;  % [1/100 m/s^2] 重力加速度，1/100 是将 cm 转为 m

headNeckLx = sind(trunk.a)*headNeck.L; % 头颈长度在 x 方向上的投影

Mnt = dictionary();

m = hand.wt+headNeck.wt+trunk.wt+upperArm.wt+foreArm.wt;
Mnt("hip") = {[
    bar.x*bar.wt*g - hip.x*bar.wt*g;
    hand.x*hand.wt*g + ...
    (shoulder.x + headNeck.CoG*headNeckLx              ) * headNeck.wt*g+ ...
    (     hip.x +    trunk.CoG*(shoulder.x -     hip.x)) * trunk.wt*g + ...
    (shoulder.x + upperArm.CoG*(   elbow.x -shoulder.x)) * upperArm.wt*g + ...
    (   elbow.x +  foreArm.CoG*(    hand.x -   elbow.x)) * foreArm.wt*g + ...
    -hip.x*m*g
]};

m = hand.wt+headNeck.wt+trunk.wt+thigh.wt+upperArm.wt+foreArm.wt;
Mnt("knee") = {[
    bar.x*bar.wt*g - knee.x*bar.wt*g;
    hand.x*hand.wt*g + ...
    (shoulder.x + headNeck.CoG*headNeckLx             ) * headNeck.wt*g + ...
    (     hip.x +    trunk.CoG*(shoulder.x-     hip.x)) * trunk.wt*g + ...
    (     hip.x +    thigh.CoG*(    knee.x-     hip.x)) * thigh.wt*g + ...
    (shoulder.x + upperArm.CoG*(   elbow.x-shoulder.x)) * upperArm.wt*g + ...
    (   elbow.x +  foreArm.CoG*(    hand.x-   elbow.x)) * foreArm.wt*g + ...
    -knee.x*m*g
]};

m = hand.wt+headNeck.wt+trunk.wt+thigh.wt+shank.wt+upperArm.wt+foreArm.wt;
Mnt("ankle") = {[
    bar.x*bar.wt*g - ankle.x*bar.wt*g;
    hand.x*hand.wt*g + ...
    (shoulder.x + headNeck.CoG*headNeckLx             ) * headNeck.wt*g + ...
    (     hip.x +    trunk.CoG*(shoulder.x-     hip.x)) * trunk.wt*g + ...
    (     hip.x +    thigh.CoG*(    knee.x-     hip.x)) * thigh.wt*g + ...
    (    knee.x +    shank.CoG*(   ankle.x -   knee.x)) * shank.wt*g + ...
    (shoulder.x + upperArm.CoG*(   elbow.x-shoulder.x)) * upperArm.wt*g + ...
    (   elbow.x +  foreArm.CoG*(    hand.x-   elbow.x)) * foreArm.wt*g + ...
    -ankle.x*m*g
]};


if ~isplot; return; end
% 绘图
figure; hold on
colors = {'r','b','c'};
k = 1;
for s = Mnt.keys'
    Mnts = Mnt(s); M = Mnts{:};
    plot(shoulder.z, M(1,:), [colors{k},'--'],'DisplayName', s+'-Bar' )
    plot(shoulder.z, M(2,:), [colors{k},':' ],'DisplayName', s+'-Body')
    plot(shoulder.z, sum(M), [colors{k},'-' ],'DisplayName', s+'-All' )
    k = k + 1;
end
set(gca,'XDir','reverse'); legend
xlabel('杠铃高度 (cm)'); ylabel('力矩 (NM)')
