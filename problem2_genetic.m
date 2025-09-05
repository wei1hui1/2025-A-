% 问题2：使用遗传算法优化FY1的飞行参数以最大化遮蔽时间
% 将原有的网格搜索+PSO算法替换为遗传算法

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
% 飞行速度：70-140 m/s
% 飞行角度：160-200度 (约2.79-3.49弧度)
% 投放时间：0-5秒
% 起爆延迟：0-5秒
global_lb = [80, 0, 0, 0];     % 全局下界
global_ub = [120, 0.13, 2, 2];   % 全局上界
num_vars = length(global_lb);

fprintf('=== 遗传算法优化开始 ===\n\n');

%% 遗传算法参数设置
tic;
fprintf('遗传算法参数设置...\n');

% 调试开关
global debug_mode;
debug_mode = true;  % 设置为false可关闭详细调试输出

% GA参数
population_size = 20;        % 种群大小（调试时减少）
max_generations = 50;        % 最大代数（调试时减少）
crossover_rate = 0.8;        % 交叉概率
mutation_rate = 0.1;         % 变异概率
elite_count = 2;             % 精英个体数量
tournament_size = 3;         % 锦标赛选择大小
num_runs = 1;                % 多次运行取最优（调试时减少）

% 存储所有运行结果
all_ga_results = [];
best_overall_fitness = -inf;
best_overall_params = [];

% 多次运行遗传算法以提高稳定性
for run = 1:num_runs
    fprintf('\n--- 第 %d 次遗传算法运行 ---\n', run);
    
    % 设置随机种子
    rng(42 + run);
    
    % 初始化种群
    population = initialize_population(population_size, global_lb, global_ub);
    
    % 评估初始种群适应度
    fitness = evaluate_population(population);
    
    % 存储优化历史
    fitness_history = zeros(max_generations, 3); % [最优, 平均, 最差]
    
    fprintf('代数\t最优适应度\t平均适应度\t速度(m/s)\t方向角(度)\n');
    fprintf('----\t----------\t----------\t----------\t----------\n');
    
    % 遗传算法主循环
    for generation = 1:max_generations
        % 记录当前代的统计信息
        [best_fitness, best_idx] = max(fitness);
        avg_fitness = mean(fitness);
        worst_fitness = min(fitness);
        
        fitness_history(generation, :) = [best_fitness, avg_fitness, worst_fitness];
        
        % 显示进度
        if mod(generation, 10) == 0 || generation == 1
            best_individual = population(best_idx, :);
            fprintf('%d\t%.6f\t%.6f\t%.2f\t%.2f\n', generation, best_fitness, avg_fitness, best_individual(1), best_individual(2) * 180 / pi);
        end
        
        % 创建新一代
        new_population = zeros(size(population));
        
        % 精英保留
        [~, elite_indices] = sort(fitness, 'descend');
        new_population(1:elite_count, :) = population(elite_indices(1:elite_count), :);
        
        % 生成剩余个体
        for i = (elite_count + 1):population_size
            % 选择父母
            parent1 = tournament_selection(population, fitness, tournament_size);
            parent2 = tournament_selection(population, fitness, tournament_size);
            
            % 交叉
            if rand < crossover_rate
                offspring = crossover(parent1, parent2, global_lb, global_ub);
            else
                offspring = parent1; % 不交叉则选择父母之一
            end
            
            % 变异
            if rand < mutation_rate
                offspring = mutate(offspring, global_lb, global_ub);
            end
            
            new_population(i, :) = offspring;
        end
        
        % 更新种群
        population = new_population;
        fitness = evaluate_population(population);
        
        % 早停条件（连续20代无改善）
        if generation > 20
            recent_best = max(fitness_history(max(1, generation-19):generation, 1));
            if recent_best - fitness_history(max(1, generation-19), 1) < 1e-6
                fprintf('早停：连续20代无显著改善\n');
                break;
            end
        end
    end
    
    % 记录本次运行的最佳结果
    [run_best_fitness, run_best_idx] = max(fitness);
    run_best_params = population(run_best_idx, :);
    all_ga_results = [all_ga_results; run_best_fitness, run_best_params];
    
    % 更新全局最佳
    if run_best_fitness > best_overall_fitness
        best_overall_fitness = run_best_fitness;
        best_overall_params = run_best_params;
    end
    
    fprintf('第 %d 次运行完成，最佳适应度: %.6f\n', run, run_best_fitness);
end

ga_time = toc;
fprintf('\n遗传算法优化完成，总用时: %.2f秒\n', ga_time);
fprintf('遗传算法最佳适应度: %.6f\n', best_overall_fitness);
fprintf('遗传算法最佳参数: [%.3f, %.3f, %.3f, %.3f]\n', best_overall_params);
fprintf('遗传算法最佳方向角: %.2f度\n', best_overall_params(2) * 180 / pi);

% 显示多次运行的统计信息
fprintf('\n=== 遗传算法多次运行统计 ===\n');
fprintf('运行次数: %d\n', num_runs);
fprintf('适应度范围: %.6f - %.6f\n', min(all_ga_results(:,1)), max(all_ga_results(:,1)));
fprintf('适应度标准差: %.6f\n', std(all_ga_results(:,1)));
fprintf('方向角范围: %.2f° - %.2f°\n', min(all_ga_results(:,3)) * 180 / pi, max(all_ga_results(:,3)) * 180 / pi);
fprintf('方向角标准差: %.2f°\n', std(all_ga_results(:,3)) * 180 / pi);

%% SQP局部精修（可选）
fprintf('\n=== SQP局部精修 ===\n');
tic;

% 设置优化选项
options = optimoptions('fmincon', ...
    'Algorithm', 'sqp', ...
    'Display', 'iter', ...
    'MaxIterations', 50, ...
    'OptimalityTolerance', 1e-6, ...
    'StepTolerance', 1e-8);

% 目标函数（最大化转为最小化）
objective = @(x) -fitness_function(x);

% 约束条件
A = []; b = []; Aeq = []; beq = [];
nonlcon = [];

% 使用GA结果作为初始点
x0 = best_overall_params;

% 执行SQP优化
try
    [sqp_result, sqp_fval, sqp_exitflag, sqp_output] = fmincon(objective, x0, A, b, Aeq, beq, global_lb, global_ub, nonlcon, options);
    sqp_fitness = -sqp_fval;
    sqp_success = true;
catch ME
    fprintf('SQP优化失败: %s\n', ME.message);
    sqp_result = best_overall_params;
    sqp_fitness = best_overall_fitness;
    sqp_success = false;
end

sqp_time = toc;
if sqp_success
    fprintf('SQP优化完成，用时: %.2f秒\n', sqp_time);
    fprintf('SQP最佳适应度: %.6f\n', sqp_fitness);
    fprintf('SQP最佳参数: [%.3f, %.3f, %.3f, %.3f]\n', sqp_result);
else
    fprintf('SQP优化失败，使用GA结果\n');
    sqp_time = 0;
end

% 选择最终结果
if sqp_success && sqp_fitness > best_overall_fitness
    final_result = sqp_result;
    final_fitness = sqp_fitness;
    final_method = 'SQP';
else
    final_result = best_overall_params;
    final_fitness = best_overall_fitness;
    final_method = 'GA';
end

fprintf('\n=== 优化总结 ===\n');
fprintf('遗传算法: %.6f (%.2fs)\n', best_overall_fitness, ga_time);
if sqp_success
    fprintf('SQP精修: %.6f (%.2fs)\n', sqp_fitness, sqp_time);
end
fprintf('最终采用: %s方法\n', final_method);
fprintf('总用时: %.2f秒\n', ga_time + sqp_time);

%% 遗传算法相关函数定义

% 初始化种群
function population = initialize_population(pop_size, lb, ub)
    num_vars = length(lb);
    population = zeros(pop_size, num_vars);
    
    for i = 1:pop_size
        for j = 1:num_vars
            population(i, j) = lb(j) + (ub(j) - lb(j)) * rand;
        end
    end
end

% 评估种群适应度
function fitness = evaluate_population(population)
    pop_size = size(population, 1);
    fitness = zeros(pop_size, 1);
    
    for i = 1:pop_size
        fitness(i) = fitness_function(population(i, :));
    end
end

% 锦标赛选择
function selected = tournament_selection(population, fitness, tournament_size)
    pop_size = size(population, 1);
    
    % 随机选择锦标赛参与者
    tournament_indices = randperm(pop_size, tournament_size);
    tournament_fitness = fitness(tournament_indices);
    
    % 选择最优个体
    [~, winner_idx] = max(tournament_fitness);
    selected = population(tournament_indices(winner_idx), :);
end

% 交叉操作（算术交叉）
function offspring = crossover(parent1, parent2, lb, ub)
    alpha = rand; % 交叉系数
    offspring = alpha * parent1 + (1 - alpha) * parent2;
    
    % 边界约束
    offspring = max(offspring, lb);
    offspring = min(offspring, ub);
end

% 变异操作（高斯变异）
function mutated = mutate(individual, lb, ub)
    num_vars = length(individual);
    mutated = individual;
    
    % 对每个基因以一定概率进行变异
    for i = 1:num_vars
        if rand < 0.3 % 30%概率变异每个基因
            % 高斯变异
            sigma = (ub(i) - lb(i)) * 0.1; % 变异强度为范围的10%
            mutated(i) = individual(i) + sigma * randn;
            
            % 边界约束
            mutated(i) = max(mutated(i), lb(i));
            mutated(i) = min(mutated(i), ub(i));
        end
    end
end

% --- 适应度函数定义（与原文件相同）---
function duration = fitness_function(params)
    % 解包参数
    flight_speed = params(1);
    flight_angle = params(2);
    drop_time = params(3);
    detonation_delay = params(4);
    
    % 调试信息：打印投掷时间和延迟时间
    global debug_mode;
    if debug_mode
        fprintf('调试 - 投掷时间: %.3f秒, 延迟时间: %.3f秒, 飞行角度: %.2f度\n', ...
                drop_time, detonation_delay, flight_angle * 180 / pi);
    end
    
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
        
        % 调试信息：打印关键位置
        if debug_mode
            fprintf('调试 - 投放点: (%.2f, %.2f, %.2f)\n', drop_pos(1), drop_pos(2), drop_pos(3));
            fprintf('调试 - 起爆点: (%.2f, %.2f, %.2f), 起爆时间: %.3f秒\n', ...
                    detonation_pos_x, detonation_pos_y, detonation_pos_z, detonation_time);
        end
        
        % 检查起爆点是否在地面以上
        if detonation_pos_z < 0
            duration = 0;
            if debug_mode
                fprintf('调试 - 起爆点在地面以下，无效方案\n');
            end
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
            if debug_mode
                fprintf('调试 - 无遮蔽效果\n');
            end
        else
            duration = t_end_obscuration - t_start_obscuration;
            if debug_mode
                fprintf('调试 - 遮蔽时长: %.6f秒 (从%.3f到%.3f秒)\n', ...
                        duration, t_start_obscuration, t_end_obscuration);
            end
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
fprintf('最大遮蔽时长: %.6f 秒\n', final_fitness);

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
% 1. 遗传算法优化过程图
figure(1);
set(gcf, 'Position', [100, 100, 1200, 800]);

subplot(2,2,1);
plot(1:size(fitness_history,1), fitness_history(:,1), 'r-', 'LineWidth', 2);
hold on;
plot(1:size(fitness_history,1), fitness_history(:,2), 'b--', 'LineWidth', 1.5);
plot(1:size(fitness_history,1), fitness_history(:,3), 'g:', 'LineWidth', 1);
xlabel('代数');
ylabel('适应度');
title('遗传算法优化过程');
legend('最优', '平均', '最差', 'Location', 'best');
grid on;

subplot(2,2,2);
if size(all_ga_results, 1) > 1
    bar(all_ga_results(:,1));
    xlabel('运行次数');
    ylabel('最佳适应度');
    title('多次运行结果对比');
    grid on;
else
    text(0.5, 0.5, '单次运行', 'HorizontalAlignment', 'center');
    title('运行结果');
end

subplot(2,2,3);
% 参数分布图
if size(all_ga_results, 1) > 1
    scatter(all_ga_results(:,2), all_ga_results(:,3)*180/pi, 50, all_ga_results(:,1), 'filled');
    xlabel('飞行速度 (m/s)');
    ylabel('飞行角度 (度)');
    title('参数空间分布');
    colorbar;
    colormap('jet');
else
    scatter(final_result(1), final_result(2)*180/pi, 100, 'r', 'filled');
    xlabel('飞行速度 (m/s)');
    ylabel('飞行角度 (度)');
    title('最优参数');
end
grid on;

subplot(2,2,4);
% 算法性能对比
method_names = {'遗传算法'};
method_fitness = [best_overall_fitness];
method_time = [ga_time];

if sqp_success
    method_names{end+1} = 'SQP精修';
    method_fitness(end+1) = sqp_fitness;
    method_time(end+1) = sqp_time;
end

yyaxis left;
bar(method_fitness);
ylabel('适应度');
yyaxis right;
plot(1:length(method_time), method_time, 'ro-', 'LineWidth', 2, 'MarkerSize', 8);
ylabel('时间 (秒)');
set(gca, 'XTickLabel', method_names);
title('算法性能对比');
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
title('问题2：遗传算法优化的飞行路径与烟幕干扰分析');
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

fprintf('\n遗传算法优化完成！可视化图表已生成。\n');
fprintf('图1显示了遗传算法优化过程和参数分布，图2显示了最优飞行路径和烟幕干扰效果。\n');
fprintf('\n遗传算法特点:\n');
fprintf('- 全局搜索能力强，避免局部最优\n');
fprintf('- 种群多样性保证了解的鲁棒性\n');
fprintf('- 多次运行提高了结果的可靠性\n');
fprintf('- 适合处理复杂的非线性优化问题\n');