function [trunk, thigh, shank, upperArm, foreArm, hand, headNeck, ...
          foot, neck, head, hip, shoulder, arm] = sportsman(height, weight)

if nargin==0; height = 180; weight = 75; end   % [cm] 身高和 [kg] 体重

%% 各部分长度占身高的百分比
%  Biomechanics and Motor Control of Human Movement p.83 Fig 4.1
trunk.L = (0.818-0.530)*height;  % 人体躯干长
thigh.L = (0.530-0.285)*height;  % 大腿骨（股骨）长
shank.L = (0.285-0.039)*height;  % 小腿骨（胫骨）长
upperArm.L = 0.186*height;       % 上臂长
foreArm.L = 0.146*height;        % 前臂长
hand.L = 0.108/2*height;         % (半)手长
arm.L = upperArm.L + foreArm.L + hand.L;
foot.L = 0.152*height;           % 脚长
neck.L = 0.052*height;           % 颈长
head.R = 0.13*height/2;          % 头部半径
hip.W = 0.191*height/2;          % 臀部宽度的一半
shoulder.W = 0.129*height;       % 肩宽的一半
headNeck.L = neck.L + 2*head.R;  % 头颈长

%% 各部分重量占总体重的百分比
%  http://www.exrx.net/Kinesiology/Segments.html
headNeck.wt = 0.0681*weight;     % 头颈重
trunk.wt = 0.4302*weight;        % 躯干重
thigh.wt = 2*0.1447*weight;      % 两大腿重
shank.wt = 2*0.0457*weight;      % 两小腿重
upperArm.wt = 2*0.0263*weight;   % 两上臂重
foreArm.wt = 2*0.015*weight;     % 两前臂重
hand.wt = 2*0.00585*weight;      % 两只手重
foot.wt = 2*0.0133*weight;       % 两只脚重
arm.wt = upperArm.wt + foreArm.wt + hand.wt;

%% 重心高度比例：从靠近身体中心的一端开始测量到该部分的其他端点的长度比例
%  http://www.exrx.net/Kinesiology/Segments.html
headNeck.CoG = 0.4922;           % 头颈
% trunk.CoG = 0.4046;            % 躯干 %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%% 数据有问题
trunk.CoG = 1- 0.4046;           % 0.5954
thigh.CoG = 0.3854;              % 大腿
shank.CoG = 0.4374;              % 小腿
upperArm.CoG = 0.5763;           % 上臂 ?
foreArm.CoG = 0.4567;            % 前臂

%% 固定的坐标
shoulder.y = shoulder.W;
hip.y = 0;
neck.y = 0;
