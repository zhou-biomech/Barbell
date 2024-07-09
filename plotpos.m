function plotpos(neck, trunk, ankle, toe, heel, knee, hip, shoulder, ...
                                                 elbow, hand, head, bar, n)

if nargin==12; n = 1:length(knee.x); end

xc = cosd(0:10:360); yc = sind(0:10:360); % 用来绘制圆

for i = n
%% 绘制除头颈外的人体关节
x1 = [ankle.x, toe.x, heel.x, ankle.x, knee.x(i), hip.x(i), hip.x(i), ...
     shoulder.x(i), shoulder.x(i), elbow.x(i), hand.x(i)];
y1 = [ankle.y, toe.y, heel.y, ankle.y, knee.y(i), hip.W   ,        0, ...
                 0, shoulder.y   , elbow.y(i), hand.y    ];
z1 = [ankle.z, toe.z, heel.z, ankle.z, knee.z(i), hip.z(i), hip.z(i), ...
     shoulder.z(i), shoulder.z(i), elbow.z(i), hand.z(i)];

plot3([x1, NaN, x1], [y1, NaN, -y1], [z1, NaN, z1], 'o-r')
hold on

%% 绘制头颈
head.bx = shoulder.x(i) + neck.L * sind(trunk.a(i));
head.bz = shoulder.z(i) + neck.L * cosd(trunk.a(i));
headx = head.bx + head.R * sind(trunk.a(i)) + head.R*xc;
headz = head.bz + head.R * cosd(trunk.a(i)) + head.R*yc;

x2 = [shoulder.x(i), head.bx, NaN, headx  ];
y2 = [            0,       0, NaN, headx*0];
z2 = [shoulder.z(i), head.bz, NaN, headz  ];

plot3(x2, y2, z2, '-r')

%% 绘制杠铃
barx = bar.R*xc + bar.x(i);
barz = bar.R*yc + bar.z(i);

x3 = [            barx, NaN,              barx, NaN, [1,1]*bar.x(i)];
y3 = [ones(size(barx)), NaN, -ones(size(barx)), NaN, -1,1]*bar.L;
z3 = [            barz, NaN,              barz, NaN, [1,1]*bar.z(i)];
plot3(x3,y3,z3, 'b')
xlabel('x'); ylabel('y'); zlabel('z')
axis image; axis([-150,150,-150,150,0,200])
hold off; view(45,30)
drawnow
end