%% 示意图：导弹 - 圆柱目标 - 烟雾球 的角锥关系
clc; clear; close all; figure('Color','w','Position',[100 100 900 600]);

% ----- 参数（可调整） -----
M = [0, 0, 0];            % 导弹位置（观察点）
T = [10, 0, 0];            % 圆柱底面圆心位置
r_t = 1.5;                % 目标半径
h_t = 3;                  % 目标高度
Rc = 2.0;                 % 烟雾球半径
C = [5, 1.0, 1.0];        % 烟雾球心位置

% 绘制圆柱（目标）
[XC,YC,ZC] = cylinder(r_t,64);
ZC = ZC * h_t;
XC = XC + T(1);
YC = YC + T(2);
ZC = ZC + T(3);
surf(XC,YC,ZC,'FaceColor',[0.2 0.6 1],'FaceAlpha',0.6,'EdgeColor','none'); hold on;
fill3(XC(1,:),YC(1,:),ZC(1,:),[0.2 0.6 1],'FaceAlpha',0.6,'EdgeColor','none');
fill3(XC(2,:),YC(2,:),ZC(2,:),[0.2 0.6 1],'FaceAlpha',0.6,'EdgeColor','none');

% 绘制烟雾球（球体）
[XS,YS,ZS] = sphere(60);
XS = Rc*XS + C(1);
YS = Rc*YS + C(2);
ZS = Rc*ZS + C(3);
surf(XS,YS,ZS,'FaceColor',[1 0.5 0],'FaceAlpha',0.35,'EdgeColor','none');

% 绘制导弹
plot3(M(1),M(2),M(3),'ro','MarkerFaceColor','r','MarkerSize',8);

% ----- 计算目标的近似最小包容锥（示意） -----
% 在目标表面采样点，得到从导弹看的方向向量集 u_k
nphi = 72; nz = 20;
phi = linspace(0,2*pi,nphi);
zvals = linspace(0,h_t,nz);
P = [];  % 目标表面采样点
for iz = 1:nz
    z0 = T(3) + zvals(iz);
    xs = r_t*cos(phi) + T(1);
    ys = r_t*sin(phi) + T(2);
    zs = ones(size(xs))*z0;
    P = [P; xs', ys', zs'];
end
% 顶面与底面边缘也采样
P = [P; (T(1)+r_t*cos(phi))', (T(2)+r_t*sin(phi))', (T(3)+h_t*ones(size(phi)))'];
P = [P; (T(1)+r_t*cos(phi))', (T(2)+r_t*sin(phi))', (T(3)+zeros(size(phi)))'];

% 单位方向向量 u_k 从导弹到目标表面
U_k = bsxfun(@rdivide, P - M, sqrt(sum((P-M).^2,2)));

% 近似最小包容方向：用方向向量均值作为轴的近似（用于示意）
u_mean = mean(U_k,1);
u_T = u_mean / norm(u_mean);
% 近似半顶角 psi_T
angles = acos( max(min(U_k * u_T',1),-1) ); % clamp
psi_T = max(angles);    % 近似半顶角（rad）

% ----- 计算烟雾球的半顶角 -----
r_CM = norm(C - M);
if r_CM <= Rc
    theta_C = pi/2;    % 导弹在球内（示意处理）
else
    theta_C = asin( Rc / r_CM );   % 半顶角（rad）
end

% ----- 画出两个锥体：目标包容锥（蓝色）与烟雾球锥（橙色） -----
% 通用函数：生成从顶点 apex 出发，轴向 axis_unit，角度 alpha，长度 L 的锥面点
make_cone = @(apex, axis_unit, alpha, L, nTheta, nPhi) ...
    cone_points(apex, axis_unit, alpha, L, nTheta, nPhi);

% 设定锥长用于可视化（延伸一定距离）
Lcone = max( norm(T-M), norm(C-M) ) * 1.4;

% 目标锥
[CXt, CYt, CZt] = make_cone(M, u_T, psi_T, Lcone, 30, 60);
surf(CXt, CYt, CZt, 'FaceColor', [0 0.4470 0.7410], 'FaceAlpha', 0.12, 'EdgeColor', 'none');

% 烟雾球锥（以球心方向为轴）
s = (C - M) / norm(C - M);
[CXs, CYs, CZs] = make_cone(M, s, theta_C, Lcone, 30, 60);
surf(CXs, CYs, CZs, 'FaceColor', [0.85 0.33 0.1], 'FaceAlpha', 0.10, 'EdgeColor', 'none');

% 视图和美化
axis equal; grid on;
xlabel('X'); ylabel('Y'); zlabel('Z');
title('示意：导弹 - 目标（圆柱） - 烟雾球 的角锥关系 (示意)');
view(35,20);
camlight; lighting gouraud;
box on;

% optionally save
% saveas(gcf,'cone_demo.png');

%% ----- 子函数：生成锥体点（返回 X,Y,Z 矩阵） -----
function [Xc,Yc,Zc] = cone_points(apex, axis_unit, alpha, L, nTheta, nPhi)
    % apex : [3] apex coordinates
    % axis_unit : [3] unit vector for cone axis
    % alpha : half-angle (rad)
    % L : length of cone
    % nTheta, nPhi : resolution
    % returns surface mesh (Xc,Yc,Zc)
    % build orthonormal basis (u,v,w), w = axis_unit
    w = axis_unit(:) / norm(axis_unit);
    % pick a vector not parallel to w
    if abs(w(1)) < 0.9
        tmp = [1;0;0];
    else
        tmp = [0;1;0];
    end
    u = cross(w, tmp); u = u / norm(u);
    v = cross(w, u);
    % generate mesh in (radius, angle) space
    thetas = linspace(0, alpha, nTheta);  % along axis angle from 0 to alpha
    phis = linspace(0, 2*pi, nPhi);
    [T,P] = meshgrid(thetas, phis);
    % for each (theta,phi) get point at distance r = L * cos(theta) along axis? 
    % better param: along axis distance t from 0..L, radius = t * tan(alpha)
    ts = linspace(0, L, nTheta);
    [T2,P2] = meshgrid(ts, phis);
    R = T2 .* tan(alpha);  % radial distance at each t
    % coordinates:
    Xc = apex(1) + w(1).*T2 + u(1).* (R .* cos(P2)) + v(1).* (R .* sin(P2));
    Yc = apex(2) + w(2).*T2 + u(2).* (R .* cos(P2)) + v(2).* (R .* sin(P2));
    Zc = apex(3) + w(3).*T2 + u(3).* (R .* cos(P2)) + v(3).* (R .* sin(P2));
end
% ----- 绘制导弹 M 点 -----
plot3(M(1),M(2),M(3),'ro','MarkerFaceColor','r','MarkerSize',6);
text(M(1)-1,M(2),M(3), 'M','FontSize',12,'Color','r','FontWeight','bold');

% 烟雾球心 C 点
plot3(C(1),C(2),C(3),'ko','MarkerFaceColor',[0.85 0.33 0.1],'MarkerSize',6);
text(C(1),C(2),C(3)+0.5, '  C','FontSize',12,'Color',[0.85 0.33 0.1],'FontWeight','bold');

% 圆柱体体心 Q 点（取底面圆心+高度一半）
Q = [T(1), T(2), T(3)+h_t/2];
plot3(Q(1),Q(2),Q(3),'bo','MarkerFaceColor','b','MarkerSize',6);
text(Q(1),Q(2),Q(3), '  Q','FontSize',12,'Color','b','FontWeight','bold');

% ----- 添加向量箭头 -----
% M -> C
quiver3(M(1),M(2),M(3), C(1)-M(1), C(2)-M(2), C(3)-M(3), ...
    0,'Color',[0.85 0.33 0.1],'LineWidth',1.8,'MaxHeadSize',0.5);

% Q -> C
quiver3(Q(1),Q(2),Q(3), C(1)-Q(1), C(2)-Q(2), C(3)-Q(3), ...
    0,'Color','k','LineWidth',1.8,'MaxHeadSize',0.5);
