function A = jointAngle(j1, j2, j3, space)
if nargin==3; space = 'xz'; end

if strcmp(inputname(1),'hip'); j1.y = j1.W; end
if strcmp(inputname(3),'hip'); j3.y = j3.W; end

n = max([length(j1.z), length(j2.z), length(j3.z)]);

u = ones(1,n);
% 计算三个关节构成的夹角 j1-j2-j3
v1 = [j1.x-j2.x .*u; j1.y-j2.y .*u; j1.z-j2.z .*u];
v2 = [j3.x-j2.x .*u; j3.y-j2.y .*u; j3.z-j2.z .*u];

ind = false(1,3);
for i = space; ind = ind|(i == 'xyz'); end
v1 = v1(ind,:); v2 = v2(ind,:);

A = acosd( min(max( dot(v1,v2) ./ sqrt(dot(v1,v1).*dot(v2,v2)),  -1),1) );