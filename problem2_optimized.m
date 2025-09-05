% 问题2：三阶段优化FY1的飞行参数以最大化遮蔽时间
% 阶段1：网格搜索确定大致最优区间
% 阶段2：PSO在最优区间内全局搜索
% 阶段3：SQP局部精修

clc;
clear;
close all;

% 设置全局变量
global M1_initial_pos FY1_initial_pos fake_target_pos true_target_pos;
global missile_speed smoke_sink_speed smoke_radius smoke_effective_duration g;
global true_target_radius true_target_height;

% --- 固定参数 ---
M1_initial_pos = [20000, 0, 2000]; % M1初始位置
FY1_initial_pos = [17800, 0, 1800]; % FY1初始位置
fake_target_pos = [0, 0, 0]; % 假目标位置
true_target_pos = [0, 200, 0]; % 真目标位置
missile_speed = 300; % 导弹速度
smoke_sink_speed = 3; % 烟幕下沉速度
smoke_effective_duration = 20; % 烟幕有效持续时间
smoke_radius = 10; % 烟幕半径
true_target_radius = 7; % 真目标半径
true_target_height = 10; % 真目标高度
g = 9.8; % 重力加速度

% 导弹到达假目标的时间
missile_to_target_time = norm(M1_initial_pos - fake_target_pos) / missile_speed;

% 优化变量范围：[飞行速度, 飞行角度, 投放时间, 起爆延迟]
% 基于调试结果，缩小搜索范围到有效区域附近
% 飞行速度：80-120 m/s
% 飞行角度：0-π 弧度 (0-180度)
% 投放时间：0-2秒
% 起爆延迟：0-2秒
global_lb = [80, 0, 0, 0];                    % 全局下界
global_ub = [120, pi, 2, 2];  % 全局上界
num_vars = length(global_lb);

fprintf('=== 三阶段优化开始 ===\n\n');

%% 阶段1：网格搜索
fprintf('阶段1：网格搜索确定最优区间...\n');
tic;

% 网格搜索参数
grid_points = [10, 15, 10, 10]; % 每个维度的网格点数
grid_results = [];

% 生成网格点（与全局搜索范围保持一致）
speed_grid = linspace(global_lb(1), global_ub(1), grid_points(1));     % 飞行速度: 80-120 m/s
angle_grid = linspace(global_lb(2), global_ub(2), grid_points(2));     % 飞行角度: 0-π 弧度
drop_time_grid = linspace(global_lb(3), global_ub(3), grid_points(3)); % 投放时间: 0-2秒
delay_grid = linspace(global_lb(4), global_ub(4), grid_points(4));     % 起爆延迟: 0-2秒

total_grid_points = prod(grid_points);
current_point = 0;
best_grid_fitness = -inf;
best_grid_params = [];

for i = 1:length(speed_grid)
    for j = 1:length(angle_grid)
        for k = 1:length(drop_time_grid)
            for l = 1:length(delay_grid)
                current_point = current_point + 1;
                params = [speed_grid(i), angle_grid(j), drop_time_grid(k), delay_grid(l)];
                fitness = fitness_function(params);
                grid_results = [grid_results; params, fitness];
                
                if fitness > best_grid_fitness
                    best_grid_fitness = fitness;
                    best_grid_params = params;
                end
                
                % 显示进度
                if mod(current_point, 100) == 0
                    fprintf('网格搜索进度: %d/%d (%.1f%%)\n', current_point, total_grid_points, 100*current_point/total_grid_points);
                end
            end
        end
    end
end

grid_time = toc;
fprintf('网格搜索完成，用时: %.2f秒\n', grid_time);
fprintf('网格搜索最佳适应度: %.4f\n', best_grid_fitness);
fprintf('网格搜索最佳参数: [%.2f, %.2f, %.2f, %.2f]\n', best_grid_params);

% 确定PSO搜索区间（基于网格搜索结果的前10%）
top_percent = 0.1;
sorted_results = sortrows(grid_results, 5, 'descend');
top_results = sorted_results(1:ceil(top_percent * size(sorted_results, 1)), 1:4);

% 计算搜索区间
pso_lb = min(top_results, [], 1);
pso_ub = max(top_results, [], 1);

% 扩展搜索区间（避免过于狭窄）
range_expansion = 0.2;
for dim = 1:4
    range_size = pso_ub(dim) - pso_lb(dim);
    if range_size < (global_ub(dim) - global_lb(dim)) * 0.1
        center = (pso_lb(dim) + pso_ub(dim)) / 2;
        half_range = (global_ub(dim) - global_lb(dim)) * 0.1 / 2;
        pso_lb(dim) = max(global_lb(dim), center - half_range);
        pso_ub(dim) = min(global_ub(dim), center + half_range);
    else
        expansion = range_size * range_expansion;
        pso_lb(dim) = max(global_lb(dim), pso_lb(dim) - expansion);
        pso_ub(dim) = min(global_ub(dim), pso_ub(dim) + expansion);
    end
end

fprintf('\nPSO搜索区间:\n');
fprintf('速度: [%.2f, %.2f]\n', pso_lb(1), pso_ub(1));
fprintf('角度: [%.2f, %.2f]\n', pso_lb(2), pso_ub(2));
fprintf('投放时间: [%.2f, %.2f]\n', pso_lb(3), pso_ub(3));
fprintf('起爆延迟: [%.2f, %.2f]\n', pso_lb(4), pso_ub(4));

%% 阶段2：PSO全局搜索
fprintf('\n阶段2：PSO全局搜索...\n');
tic;

% PSO参数设置（优化版：减少计算量）
num_particles = 15;  % 减少粒子数量
max_iterations = 30; % 减少迭代次数
w = 0.9; % 惯性权重
c1 = 2.0; % 个体学习因子
c2 = 2.0; % 社会学习因子
num_runs = 2; % 减少运行次数

% 多次运行PSO以提高稳定性
best_overall_fitness = -inf;
best_overall_params = [];
all_results = [];

for run = 1:num_runs
    % 为每次运行设置不同的随机种子
    rng(42 + run);
    
    % 初始化粒子群（在确定的区间内）
    particles = pso_lb + (pso_ub - pso_lb) .* rand(num_particles, num_vars);
    velocities = zeros(num_particles, num_vars);
    pbest = particles;
    pbest_fitness = -inf(num_particles, 1);
    gbest = particles(1, :);
    gbest_fitness = -inf;
    
    % 将网格搜索的最佳结果加入粒子群
    particles(1, :) = best_grid_params;
    
    % 存储优化历史
    fitness_history = zeros(max_iterations, 1);
    
    if run == 1
        fprintf('迭代\t最佳适应度\n');
        fprintf('----\t----------\n');
    end

    % PSO主循环
    for iter = 1:max_iterations
        % 评估所有粒子的适应度
        for i = 1:num_particles
            fitness = fitness_function(particles(i, :));
            
            % 更新个体最佳
            if fitness > pbest_fitness(i)
                pbest_fitness(i) = fitness;
                pbest(i, :) = particles(i, :);
            end
            
            % 更新全局最佳
            if fitness > gbest_fitness
                gbest_fitness = fitness;
                gbest = particles(i, :);
            end
        end
        
        % 更新粒子速度和位置
        for i = 1:num_particles
            % 速度更新
            r1 = rand(1, num_vars);
            r2 = rand(1, num_vars);
            velocities(i, :) = w * velocities(i, :) + ...
                              c1 * r1 .* (pbest(i, :) - particles(i, :)) + ...
                              c2 * r2 .* (gbest - particles(i, :));
            
            % 位置更新
            particles(i, :) = particles(i, :) + velocities(i, :);
            
            % 边界约束
            particles(i, :) = max(particles(i, :), pso_lb);
            particles(i, :) = min(particles(i, :), pso_ub);
        end
        
        % 记录历史
        fitness_history(iter) = gbest_fitness;
        
        % 显示进度（只在第一次运行时显示）
        if run == 1 && (mod(iter, 10) == 0 || iter == 1)
            fprintf('%d\t%.4f\n', iter, gbest_fitness);
        end
        
        % 更新惯性权重（线性递减）
        w = 0.9 - 0.5 * iter / max_iterations;
    end
    
    % 收集本次运行结果
    all_results = [all_results; gbest_fitness, gbest];
    
    % 更新全局最佳
    if gbest_fitness > best_overall_fitness
        best_overall_fitness = gbest_fitness;
        best_overall_params = gbest;
    end
end

pso_time = toc;
fprintf('PSO搜索完成，用时: %.2f秒\n', pso_time);
fprintf('PSO最佳适应度: %.4f\n', best_overall_fitness);
fprintf('PSO最佳参数: [%.2f, %.2f, %.2f, %.2f]\n', best_overall_params);

% 显示多次运行的统计信息
fprintf('\n=== PSO多次运行统计 ===\n');
fprintf('运行次数: %d\n', num_runs);
fprintf('适应度范围: %.4f - %.4f\n', min(all_results(:,1)), max(all_results(:,1)));
fprintf('适应度标准差: %.4f\n', std(all_results(:,1)));

pso_result = best_overall_params;
pso_fitness = best_overall_fitness;

%% 阶段3：SQP局部精修
fprintf('\n阶段3：SQP局部精修...\n');
tic;

% 设置优化选项
options = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...
    'Display', 'iter', ...
    'MaxIterations', 100, ...
    'OptimalityTolerance', 1e-6, ...
    'StepTolerance', 1e-8);

% 目标函数（最大化转为最小化）
objective = @(x) -fitness_function(x);

% 约束条件
A = []; b = []; Aeq = []; beq = [];
nonlcon = [];

% 使用PSO结果作为初始点
x0 = pso_result;

% 执行SQP优化
try
    [sqp_result, sqp_fval, sqp_exitflag, sqp_output] = fmincon(objective, x0, A, b, Aeq, beq, global_lb, global_ub, nonlcon, options);
    sqp_fitness = -sqp_fval;
    sqp_success = true;
catch ME
    fprintf('SQP优化失败: %s\n', ME.message);
    sqp_result = pso_result;
    sqp_fitness = pso_fitness;
    sqp_success = false;
end

sqp_time = toc;
if sqp_success
    fprintf('SQP优化完成，用时: %.2f秒\n', sqp_time);
    fprintf('SQP最佳适应度: %.4f\n', sqp_fitness);
    fprintf('SQP最佳参数: [%.2f, %.2f, %.2f, %.2f]\n', sqp_result);
else
    fprintf('SQP优化失败，使用PSO结果\n');
    sqp_time = 0;
end

% 选择最终结果
if sqp_success && sqp_fitness > pso_fitness
    final_result = sqp_result;
    final_fitness = sqp_fitness;
    final_method = 'SQP';
else
    final_result = pso_result;
    final_fitness = pso_fitness;
    final_method = 'PSO';
end

fprintf('\n=== 三阶段优化总结 ===\n');
fprintf('网格搜索: %.4f (%.2fs)\n', best_grid_fitness, grid_time);
fprintf('PSO搜索: %.4f (%.2fs)\n', pso_fitness, pso_time);
if sqp_success
    fprintf('SQP精修: %.4f (%.2fs)\n', sqp_fitness, sqp_time);
end
fprintf('最终采用: %s方法\n', final_method);
fprintf('总用时: %.2f秒\n', grid_time + pso_time + sqp_time);



% --- 适应度函数定义 ---
function duration = fitness_function(params)
    % 解包参数
    flight_speed = params(1);
    flight_angle = params(2);
    drop_time = params(3);
    detonation_delay = params(4);
    
    % 全局变量访问
    global M1_initial_pos FY1_initial_pos fake_target_pos true_target_pos;
    global missile_speed smoke_sink_speed smoke_radius smoke_effective_duration g;
    global true_target_radius true_target_height;
    
    try
        % 1. 导弹M1的运动轨迹
        missile_direction = (fake_target_pos - M1_initial_pos) / norm(fake_target_pos - M1_initial_pos);
        missile_pos = @(t) M1_initial_pos + missile_direction * missile_speed * t;
        
        % 2. 无人机FY1的运动轨迹 (在X-Y平面内飞行)
        fy1_direction = [cos(flight_angle), sin(flight_angle), 0];
        fy1_pos = @(t) FY1_initial_pos + fy1_direction .* flight_speed .* t;
        
        % 3. 烟幕弹投放点
        drop_pos = fy1_pos(drop_time);
        drop_velocity = fy1_direction * flight_speed;
        
        % 4. 烟幕弹起爆点
        detonation_time = drop_time + detonation_delay;
        fall_time = detonation_delay;
        detonation_pos_x = drop_pos(1) + drop_velocity(1) * fall_time;
        detonation_pos_y = drop_pos(2) + drop_velocity(2) * fall_time;
        detonation_pos_z = drop_pos(3) - 0.5 * g * fall_time^2;
        detonation_pos = [detonation_pos_x, detonation_pos_y, detonation_pos_z];
        
        % 检查起爆点是否在地面以上
        if detonation_pos_z < 0
            duration = 0;
            return;
        end
        
        % 5. 烟幕云团中心位置
        smoke_center_pos = @(t) detonation_pos - [0, 0, smoke_sink_speed * (t - detonation_time)];
        
        % 6. 计算有效遮蔽时间（使用改进的遮蔽判断逻辑）
        t_start_obscuration = -1;
        t_end_obscuration = -1;
        
        % 模拟时间从起爆开始到烟幕失效（提高精度）
        time_step = 0.001;  % 提高时间精度到1毫秒
        for t = detonation_time : time_step : (detonation_time + smoke_effective_duration)
            
            % 当前时刻的导弹位置
            current_missile_pos = missile_pos(t);
            
            % 当前时刻烟幕云团中心位置
            current_smoke_center = smoke_center_pos(t);
            
            % 使用圆柱体目标的遮蔽判断逻辑
            intersects = cylinder_obscuration_check(current_missile_pos, current_smoke_center, smoke_radius, true_target_pos, true_target_radius, true_target_height);
            
            if intersects
                if t_start_obscuration == -1
                    t_start_obscuration = t;
                end
                t_end_obscuration = t;
            end
        end
        
        % 计算遮蔽时长
        if t_start_obscuration == -1
            duration = 0;
        else
            duration = t_end_obscuration - t_start_obscuration;
        end
        
    catch
        duration = 0;
    end
end

% 基于圆柱体目标的遮蔽判断函数（使用theta1和theta2角度比较）
function is_obscured = cylinder_obscuration_check(missile_pos, smoke_center, smoke_radius, cylinder_center, cylinder_radius, cylinder_height)
    % 计算导弹到烟雾球心的向量
    missile_to_smoke = smoke_center(:)' - missile_pos(:)';
    
    % 计算半锥角theta1：导弹到烟雾球心连线与烟雾球表面的夹角
    distance_to_smoke = norm(missile_to_smoke);
    if distance_to_smoke <= smoke_radius
        % 导弹在烟雾球内，完全被遮蔽
        is_obscured = true;
        return;
    end
    
    sin_theta1 = smoke_radius / distance_to_smoke;
    sin_theta1 = min(sin_theta1, 1); % 确保在有效范围内
    theta1 = asin(sin_theta1);
    
    % 在圆柱体表面采样点进行遮蔽检查
    num_points_height = 8; % 高度方向采样点数
    num_points_circumference = 12; % 圆周方向采样点数
    
    % 圆柱底面中心和顶面中心
    cylinder_bottom = cylinder_center;
    cylinder_top = cylinder_center + [0, 0, cylinder_height];
    
    is_obscured = true; % 假设被遮蔽，直到找到未被遮蔽的点
    
    % 检查圆柱侧面的点
    for h_idx = 1:num_points_height
        height_ratio = (h_idx - 1) / (num_points_height - 1);
        current_height = cylinder_bottom(3) + height_ratio * cylinder_height;
        
        for c_idx = 1:num_points_circumference
            angle = 2 * pi * (c_idx - 1) / num_points_circumference;
            
            % 圆柱表面点坐标
            surface_point = [
                cylinder_center(1) + cylinder_radius * cos(angle),
                cylinder_center(2) + cylinder_radius * sin(angle),
                current_height
            ];
            
            % 计算导弹到圆柱表面点的向量
            missile_to_surface = surface_point(:)' - missile_pos(:)';
             
             % 计算theta2：导弹-表面点向量与导弹-烟雾球心向量的夹角
             cos_theta2 = dot(missile_to_surface, missile_to_smoke) / (norm(missile_to_surface) * norm(missile_to_smoke));
             cos_theta2 = max(-1, min(1, cos_theta2)); % 限制在[-1,1]范围内
             theta2 = acos(cos_theta2);
            
            % 如果theta2 >= theta1，说明该点未被遮蔽
             if theta2 >= theta1
                 is_obscured = false;
                 return;
             end
        end
    end
    
    % 检查圆柱顶面和底面的点
    for face = 1:2
        if face == 1
            face_center = cylinder_bottom;
        else
            face_center = cylinder_top;
        end
        
        % 在圆形面上取点
        num_radial = 5; % 径向5个层次
        for r_idx = 0:num_radial
            if r_idx == 0
                % 中心点
                surface_point = face_center;
                
                missile_to_surface = surface_point(:)' - missile_pos(:)';
                cos_theta2 = dot(missile_to_surface, missile_to_smoke) / (norm(missile_to_surface) * norm(missile_to_smoke));
                cos_theta2 = max(-1, min(1, cos_theta2));
                theta2 = acos(cos_theta2);
                
                if theta2 >= theta1
                    is_obscured = false;
                    return;
                end
            else
                radius_ratio = r_idx / num_radial;
                current_radius = cylinder_radius * radius_ratio;
                
                for c_idx = 1:num_points_circumference
                    angle = 2 * pi * (c_idx - 1) / num_points_circumference;
                    
                    surface_point = [
                        face_center(1) + current_radius * cos(angle),
                        face_center(2) + current_radius * sin(angle),
                        face_center(3)
                    ];
                    
                    missile_to_surface = surface_point(:)' - missile_pos(:)';
                cos_theta2 = dot(missile_to_surface, missile_to_smoke) / (norm(missile_to_surface) * norm(missile_to_smoke));
                cos_theta2 = max(-1, min(1, cos_theta2));
                theta2 = acos(cos_theta2);
                
                if theta2 >= theta1
                     is_obscured = false;
                     return;
                 end
                end
            end
        end
    end
end

% --- 输出最优结果 ---
fprintf('\n=== 最终优化结果 ===\n');
fprintf('最优飞行速度: %.3f m/s\n', final_result(1));
fprintf('最优飞行角度: %.3f 度\n', final_result(2) * 180 / pi);
fprintf('最优投放时间: %.3f 秒\n', final_result(3));
fprintf('最优起爆延迟: %.3f 秒\n', final_result(4));
fprintf('最大遮蔽时长: %.5f 秒\n', final_fitness);

% 计算最优方案的详细信息
opt_flight_speed = final_result(1);
opt_flight_angle = final_result(2);
opt_drop_time = final_result(3);
opt_detonation_delay = final_result(4);

% 重新计算最优方案的轨迹
missile_direction = (fake_target_pos - M1_initial_pos) / norm(fake_target_pos - M1_initial_pos);
opt_fy1_direction = [cos(opt_flight_angle), sin(opt_flight_angle), 0];
opt_drop_pos = FY1_initial_pos + opt_fy1_direction .* [opt_flight_speed * opt_drop_time, opt_flight_speed * opt_drop_time, 0];
opt_detonation_time = opt_drop_time + opt_detonation_delay;
opt_detonation_pos = opt_drop_pos + opt_fy1_direction .* [opt_flight_speed * opt_detonation_delay, opt_flight_speed * opt_detonation_delay, 0];
opt_detonation_pos(3) = opt_detonation_pos(3) - 0.5 * g * opt_detonation_delay^2;

fprintf('\n=== 关键位置信息 ===\n');
fprintf('最优投放点: (%.3f, %.3f, %.3f)\n', opt_drop_pos(1), opt_drop_pos(2), opt_drop_pos(3));
fprintf('最优起爆点: (%.3f, %.3f, %.3f)\n', opt_detonation_pos(1), opt_detonation_pos(2), opt_detonation_pos(3));
fprintf('最优起爆时刻: %.3f 秒\n', opt_detonation_time);

% --- 可视化结果 ---
% 1. 优化过程图
figure(1);
subplot(2,2,1);
if ~isempty(grid_results)
    scatter3(grid_results(:,1), grid_results(:,2), grid_results(:,5), 20, grid_results(:,5), 'filled');
    xlabel('飞行速度 (m/s)');
    ylabel('飞行角度 (rad)');
    zlabel('适应度');
    title('网格搜索结果');
    colorbar;
end

subplot(2,2,2);
plot(1:max_iterations, fitness_history, 'b-', 'LineWidth', 2);
xlabel('迭代次数');
ylabel('最优遮蔽时长 (秒)');
title('PSO优化过程');
grid on;

subplot(2,2,3);
bar([best_grid_fitness, pso_fitness, final_fitness]);
set(gca, 'XTickLabel', {'网格搜索', 'PSO', '最终结果'});
ylabel('适应度');
title('各阶段优化结果对比');
grid on;

subplot(2,2,4);
bar([grid_time, pso_time, sqp_time]);
set(gca, 'XTickLabel', {'网格搜索', 'PSO', 'SQP'});
ylabel('时间 (秒)');
title('各阶段用时对比');
grid on;

% 2. 3D轨迹图
figure(2);
set(gcf, 'Position', [100, 100, 1200, 800]);
hold on;
grid on;
axis equal;
xlabel('X (m)');
ylabel('Y (m)');
zlabel('Z (m)');
title('问题2：最优飞行路径与烟幕干扰分析');
view(30, 20);

% 绘制目标
plot3(fake_target_pos(1), fake_target_pos(2), fake_target_pos(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
text(fake_target_pos(1), fake_target_pos(2), fake_target_pos(3) + 100, '假目标', 'FontSize', 10);

[x,y,z] = cylinder(true_target_radius);
z = z * true_target_height;
surf(x + true_target_pos(1), y + true_target_pos(2), z + true_target_pos(3), 'FaceColor', 'g', 'FaceAlpha', 0.3);
text(true_target_pos(1), true_target_pos(2), true_target_pos(3) + 50, '真目标', 'FontSize', 10);

% 绘制导弹轨迹
t_sim_end = opt_detonation_time + smoke_effective_duration + 5;
t_points = 0:0.1:t_sim_end;
missile_traj = zeros(length(t_points), 3);
for i = 1:length(t_points)
    missile_traj(i, :) = M1_initial_pos + missile_direction * missile_speed * t_points(i);
end
plot3(missile_traj(:,1), missile_traj(:,2), missile_traj(:,3), 'r--', 'LineWidth', 1.5);
plot3(M1_initial_pos(1), M1_initial_pos(2), M1_initial_pos(3), 'r^', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
text(M1_initial_pos(1), M1_initial_pos(2), M1_initial_pos(3), ' M1起点');

% 绘制最优无人机轨迹
fy1_t_points = 0:0.1:opt_drop_time;
fy1_traj = zeros(length(fy1_t_points), 3);
for i = 1:length(fy1_t_points)
    fy1_traj(i, :) = FY1_initial_pos + opt_fy1_direction * opt_flight_speed * fy1_t_points(i);
end
plot3(fy1_traj(:,1), fy1_traj(:,2), fy1_traj(:,3), 'b-', 'LineWidth', 2);
plot3(FY1_initial_pos(1), FY1_initial_pos(2), FY1_initial_pos(3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
text(FY1_initial_pos(1), FY1_initial_pos(2), FY1_initial_pos(3), ' FY1起点');

% 绘制最优投放点和起爆点
plot3(opt_drop_pos(1), opt_drop_pos(2), opt_drop_pos(3), 'm*', 'MarkerSize', 10);
text(opt_drop_pos(1), opt_drop_pos(2), opt_drop_pos(3), ' 最优投放点');
plot3(opt_detonation_pos(1), opt_detonation_pos(2), opt_detonation_pos(3), 'ro', 'MarkerSize', 12, 'MarkerFaceColor', 'r');
text(opt_detonation_pos(1), opt_detonation_pos(2), opt_detonation_pos(3), ' 最优起爆点');

% 绘制最优方案的烟幕效果
smoke_center_opt = @(t) opt_detonation_pos - [0, 0, smoke_sink_speed * (t - opt_detonation_time)];
t_mid = opt_detonation_time + smoke_effective_duration / 2;
smoke_center_mid = smoke_center_opt(t_mid);
[sx,sy,sz] = sphere;
surf(sx*smoke_radius+smoke_center_mid(1), sy*smoke_radius+smoke_center_mid(2), sz*smoke_radius+smoke_center_mid(3), 'FaceColor', 'k', 'FaceAlpha', 0.4, 'EdgeColor', 'none');

legend('假目标', '真目标', 'M1轨迹', 'M1起点', '最优FY1轨迹', 'FY1起点', '最优投放点', '最优起爆点', 'Location', 'northeast');

% 设置坐标轴范围
xlim([0, 22000]);
ylim([-2000, 2000]);
zlim([0, 2500]);

fprintf('\n三阶段优化完成！可视化图表已生成。\n');
fprintf('图1显示了各阶段优化过程和结果对比，图2显示了最优飞行路径和烟幕干扰效果。\n');
fprintf('\n性能提升总结:\n');
fprintf('网格搜索 -> PSO: %.2f%% 提升\n', (pso_fitness - best_grid_fitness) / best_grid_fitness * 100);
if sqp_success && sqp_fitness > pso_fitness
    fprintf('PSO -> SQP: %.2f%% 提升\n', (sqp_fitness - pso_fitness) / pso_fitness * 100);
end
fprintf('总体提升: %.2f%% (相对于网格搜索)\n', (final_fitness - best_grid_fitness) / best_grid_fitness * 100);