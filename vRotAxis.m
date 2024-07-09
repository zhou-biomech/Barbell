function v = vRotAxis(v, theta, ax)
% 向量 v 绕轴 ax（向量）旋转 theta 度
% Reference: https://en.wikipedia.org/wiki/Rodrigues%27_rotation_formula

v = v*cosd(theta)+cross(ax,v)*sind(theta)+ax*(dot(ax,v))*(1-cosd(theta));