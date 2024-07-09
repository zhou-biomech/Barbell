function [Fc, Fs, Fm] = L5S1Force(thetaH, T, K, M, m, shoulder, isplot)

if nargin==6; isplot = true; end

g = 9.8;                        % [m/s^2] 重力加速度
mmHg = 133.32;                  % [Pa/mm] 1 mgHg 对应的压强
E = 0.0475;                     % [m    ] 背部背棘肌力臂
A = 0.0465;                     % [m^2  ] 腹部横隔膜面积

beta = -17.5 - 0.12*T + 0.23*K + 0.0012*T.*K + 0.005*T.^2 - 0.00075*K.^2;
alpha = 40 + beta;

D = 0.067 + 0.082*sind(thetaH); % [m    ] 腹部有效力臂
P = 1e-4*(0.6516 - 0.005447*thetaH).*(M*100/g).^1.8; 
P = min(max(P,0),150);          % [mmHg ] 腹压
Fa = P*mmHg*A;                  % [N    ] 腹压有效作用力

Fm = (M - Fa.*D)/E;             % [N    ] 背部背棘肌张力
Fs = m*g*sind(alpha);           % [N    ] L5/S1 剪力
Fc = Fm - Fa + m*g*cosd(alpha); % [N    ] L5/S1 压力

if ~isplot; return; end

figure; hold on
plot(shoulder.z, Fc, 'DisplayName', 'compression');
plot(shoulder.z, Fs, 'DisplayName', 'shear')
set(gca,'XDir','reverse'); legend
xlabel('杠铃高度 (cm)'); ylabel('L5/S1 受力 (N)')