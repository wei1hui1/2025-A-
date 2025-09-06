%% ------------------- 无人机干扰导弹优势评分随高度变化程序 -------------------
clc; clear; close all;

%% ----------------- 参数设置 -----------------
h_idx = 18; % 高度索引

% 导弹速度
v_m = 300;  % m/s

% 假目标 O (原点)
O = [0,0,0];

% 水平/垂直评分参数
gamma = 0.6;   % 垂直权重
kh = 500;     % 水平尺度 (m)
kv = 100;      % 垂直尺度 (m)

% 无人机坐标 (FY1..FY5)
UAVs = [
    17800,    0, 1800.1;   % FY1
    12000, 1400, 1400.1;   % FY2
     6000,-3000,  700.1;   % FY3
    11000, 2000, 1800.1;   % FY4
    13000,-2000, 1300.1    % FY5
];

UAV_names = {'FY1','FY2','FY3','FY4','FY5'};

% 导弹初始位置 (M1..M3)
Missiles0 = [
    20000,   0, 2000;   % M1
    19000, 600, 2100;   % M2
    18000,-600, 1900    % M3
];
Mis_names = {'M1','M2','M3'};

% 高度向量
heights = 0:100:2200;
numH = length(heights);

numUAV = size(UAVs,1);
numMis = size(Missiles0,1);

% 输出矩阵预分配
Missiles_h = NaN(numH, numMis,3);
Times_h = NaN(numH, numMis);
Scores_h = NaN(numH, numUAV, numMis);

%% ----------------- 导弹单位速度向量 -----------------
Vms = zeros(numMis,3);
for k = 1:numMis
    em = (O - Missiles0(k,:));
    em_unit = em / norm(em);
    Vms(k,:) = v_m * em_unit;
end

%% ----------------- 高度对应导弹位置和评分计算 -----------------
for ih = 1:numH
    h = heights(ih);
    for k = 1:numMis
        M0 = Missiles0(k,:);
        Vm = Vms(k,:);
        vz = Vm(3);
        z0 = M0(3);
        % 求解 t
        if abs(vz) < 1e-9
            if abs(h - z0) < 1e-6
                t = 0;
            else
                t = NaN;
            end
        else
            t = (h - z0)/vz;
            if t < 0
                t = NaN;
            end
        end
        Times_h(ih,k) = t;
        if ~isnan(t)
            pos = M0 + Vm * t;
            pos(3) = h; % 保证高度
            Missiles_h(ih,k,:) = pos;
            
            % 计算无人机评分
            for iu = 1:numUAV
                u = UAVs(iu,:);
                dh = hypot(u(1)-pos(1), u(2)-pos(2));
                dv = abs(u(3)-pos(3));
                fh = 1 / (1 + (dh/kh)^2);
                fv = 1 / (1 + (dv/kv)^2);
                Scores_h(ih, iu, k) = 100 * (gamma*fv + (1-gamma)*fh);
            end
        end
    end
end

%% 计算最优航向角

h = heights(h_idx);
U = UAVs(1,:);  % FY1位置
M_current = squeeze(Missiles_h(h_idx,:,:)); % 3x3, 3个导弹在当前高度
% 使用该无人机在当前高度的评分作为权重
weights = squeeze(Scores_h(h_idx, iu, :))'; % 1x3
theta_vec = linspace(0,2*pi,360); % 航向角搜索范围
D_vec = zeros(size(theta_vec));

for i = 1:length(theta_vec)
    theta = theta_vec(i);
    % 无人机沿theta方向前进一定距离d
    d = 1000; % 假设沿航向移动1000米
    U_new = U + d * [cos(theta), sin(theta), 0]; % 保持同高度
    
    % 计算到三导弹的加权距离之和
    dist = sqrt( sum((M_current - U_new).^2,2) ); % 3x1
    D_vec(i) = sum(weights .* dist');
end

% 找到最优角度
[~, idx_min] = min(D_vec);
theta_opt = theta_vec(idx_min);
fprintf('最优航向角 theta* = %.2f deg\n', rad2deg(theta_opt));

