% 单无人机对三枚导弹的烟雾弹干扰优化
% 基于3x3的01矩阵决定投放策略

clc;
clear;
close all;

% 设置全局变量
global M1_initial_pos M2_initial_pos M3_initial_pos FY1_initial_pos;
global fake_target_pos true_target_pos missile_speed smoke_sink_speed;
global smoke_radius smoke_effective_duration g true_target_radius true_target_height;
global interference_matrix;

% --- 固定参数 ---
M1_initial_pos = [20000, 0, 2000]; % M1初始位置
M2_initial_pos = [19000, 600, 2100]; % M2初始位置
M3_initial_pos = [18000, -600, 1900]; % M3初始位置

FY1_initial_pos = [17800, 0, 1800]; % FY1初始位置
%FY1_initial_pos = [12000, 1400, 1400]; % FY2初始位置
%FY1_initial_pos = [6000, -3000, 700]; % FY3初始位置
%FY1_initial_pos = [11000, 2000, 1800];% FY4初始位置
%FY1_initial_pos = [13000, -2000, 1300];% FY5初始位置

fake_target_pos = [0, 0, 0]; % 假目标位置
true_target_pos = [0, 200, 0]; % 真目标位置
missile_speed = 300; % 导弹速度
smoke_sink_speed = 3; % 烟幕下沉速度
smoke_effective_duration = 20; % 烟幕有效持续时间
smoke_radius = 10; % 烟幕半径
true_target_radius = 7; % 真目标半径
true_target_height = 10; % 真目标高度
g = 9.8; % 重力加速度

% 定义3x3干扰矩阵 (烟雾弹j对导弹k的干扰)
% 行：烟雾弹编号(1-3)，列：导弹编号(1-3)
% 1表示投放，0表示不投放
interference_matrix = [
    1, 0, 0;  % 第1枚烟雾弹对M1,M2,M3的干扰
    1, 0, 0;  % 第2枚烟雾弹对M1,M2,M3的干扰  
    0, 0, 1   % 第3枚烟雾弹对M1,M2,M3的干扰
];

fprintf('=== 单无人机对三枚导弹的烟雾弹干扰优化 ===\n\n');
fprintf('干扰矩阵:\n');
disp(interference_matrix);

% 计算需要投放的烟雾弹数量
num_bombs = sum(sum(interference_matrix > 0, 2) > 0);
fprintf('需要投放的烟雾弹数量: %d\n', num_bombs);

% 根据干扰矩阵确定优化变量维数
% 基础变量：[飞行速度, 飞行角度]
% 对每枚需要投放的烟雾弹：[投放时间, 起爆延迟]
base_vars = 2; % 飞行速度和角度
vars_per_bomb = 2; % 每枚弹的投放时间和起爆延迟
total_vars = base_vars + num_bombs * vars_per_bomb;

fprintf('优化变量总数: %d\n', total_vars);
fprintf('变量构成: [飞行速度, 飞行角度');
for i = 1:num_bombs
    fprintf(', 投放时间%d, 起爆延迟%d', i, i);
end
fprintf(']\n\n');

% 设置优化变量的上下界
global_lb = [80, 0]; % 飞行速度下界, 飞行角度下界
global_ub = [120, 0.2]; % 飞行速度上界, 飞行角度上界

% 为每枚烟雾弹添加约束
for i = 1:num_bombs
    if i == 1
        global_lb = [global_lb, 0, 0.1]; % 第1枚：投放时间下界, 起爆延迟下界
        global_ub = [global_ub, 2, 2]; % 第1枚：投放时间上界, 起爆延迟上界
    else
        global_lb = [global_lb, 1, 0.1]; % 后续枚：投放间隔下界(>=1s), 起爆延迟下界
        global_ub = [global_ub, 2, 2]; % 后续枚：投放间隔上界, 起爆延迟上界
    end
end

num_vars = length(global_lb);

fprintf('=== 遗传算法优化开始 ===\n\n');

%% 遗传算法参数设置
tic;
fprintf('遗传算法参数设置...\n');

% 调试开关
global debug_mode;
debug_mode = false; % 关闭详细调试输出以提高性能

% GA参数
population_size = 30; % 增大种群大小以更好利用已知解
max_generations = 50; % 增加最大代数
crossover_rate = 0.8; % 交叉概率
mutation_rate = 0.5; % 适当提高变异概率
elite_count = 5; % 增加精英个体数量
tournament_size = 3; % 锦标赛选择大小
num_runs = 10; % 减少运行次数，因为有了好的初始解

% 存储所有运行结果
all_ga_results = [];
best_overall_fitness = -inf;
best_overall_params = [];

% 多次运行遗传算法以提高稳定性
for run = 1:num_runs
    fprintf('\n--- 第 %d 次遗传算法运行 ---\n', run);
    
    % 设置随机种子
    %rng(42 + run);
    
    % 初始化种群（可选择是否使用已知可行解）
    use_known_solution = false; % 设置为true使用已知解，false使用随机初始化
    
    if use_known_solution
        known_solution = [124.444, 130.909*pi/180, 13.818, 3.333, 1, 1, 1, 1]; % 已知可行解
        if length(known_solution) == length(global_lb)
            population = initialize_population_with_solution(population_size, global_lb, global_ub, known_solution);
            fprintf('使用已知可行解初始化种群\n');
        else
            fprintf('警告：已知解维数不匹配，使用随机初始化\n');
            population = initialize_population(population_size, global_lb, global_ub);
        end
    else
        fprintf('使用随机初始化种群\n');
        population = initialize_population(population_size, global_lb, global_ub);
    end
    
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
        if mod(generation, 30) == 0 || generation == 1
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
        
        % 早停条件（连续40代无改善）
        if generation > 40
            recent_best = max(fitness_history(max(1, generation-39):generation, 1));
            if recent_best - fitness_history(max(1, generation-39), 1) < 1e-6
                fprintf('早停：连续40代无显著改善\n');
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
fprintf('遗传算法最佳适应度: %.6f秒\n', best_overall_fitness);
fprintf('遗传算法最佳参数: ');
for i = 1:length(best_overall_params)
    fprintf('%.3f ', best_overall_params(i));
end
fprintf('\n');
fprintf('遗传算法最佳方向角: %.2f度\n', best_overall_params(2) * 180 / pi);

% 显示多次运行的统计信息
fprintf('\n=== 遗传算法多次运行统计 ===\n');
fprintf('运行次数: %d\n', num_runs);
fprintf('适应度范围: %.6f - %.6f\n', min(all_ga_results(:,1)), max(all_ga_results(:,1)));
fprintf('适应度标准差: %.6f\n', std(all_ga_results(:,1)));

%% 输出最终结果
final_result = best_overall_params;
final_fitness = best_overall_fitness;

fprintf('\n=== 单无人机多导弹干扰策略结果 ===\n');
fprintf('最优飞行速度: %.3f m/s\n', final_result(1));
fprintf('最优飞行角度: %.3f 度\n', final_result(2) * 180 / pi);

% 输出每枚烟雾弹的参数
for i = 1:num_bombs
    drop_time_idx = 2 + (i-1)*2 + 1;
    delay_idx = 2 + (i-1)*2 + 2;
    fprintf('第%d枚烟雾弹投放时间: %.3f 秒\n', i, final_result(drop_time_idx));
    fprintf('第%d枚烟雾弹起爆延迟: %.3f 秒\n', i, final_result(delay_idx));
end

fprintf('对所有导弹的总遮蔽时长: %.5f 秒\n', final_fitness);

% 计算各导弹的详细遮蔽信息
[missile_obscuration_times, total_obscuration] = calculate_missile_obscuration_details(final_result);

fprintf('\n=== 各导弹遮蔽效果分析 ===\n');
for missile_idx = 1:3
    fprintf('导弹M%d遮蔽时间: %.5f 秒\n', missile_idx, missile_obscuration_times(missile_idx));
end
fprintf('总遮蔽时间: %.5f 秒\n', total_obscuration);

fprintf('\n单无人机多导弹干扰策略优化完成！\n');
fprintf('总用时: %.2f秒\n', ga_time);

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

% 使用已知可行解初始化种群
function population = initialize_population_with_solution(pop_size, lb, ub, known_solution)
    num_vars = length(lb);
    population = zeros(pop_size, num_vars);
    
    % 第一个个体使用已知可行解
    if length(known_solution) == num_vars
        population(1, :) = known_solution;
        start_idx = 2;
    else
        start_idx = 1;
    end
    
    % 20%的个体在已知解附近生成（局部搜索）
    local_count = min(floor(pop_size * 0.2), pop_size - start_idx + 1);
    for i = start_idx:(start_idx + local_count - 1)
        % 在已知解附近添加小幅随机扰动
        perturbation = 0.1 * (ub - lb) .* (2 * rand(1, num_vars) - 1);
        population(i, :) = known_solution + perturbation;
        
        % 确保在边界内
        population(i, :) = max(population(i, :), lb);
        population(i, :) = min(population(i, :), ub);
    end
    
    % 剩余个体随机初始化
    for i = (start_idx + local_count):pop_size
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
        fitness(i) = fitness_function_multi_missile(population(i, :));
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

% 多导弹干扰的适应度函数
function total_obscuration_time = fitness_function_multi_missile(params)
    global interference_matrix;
    
    try
        % 计算对所有导弹的总遮蔽时间
        [missile_times, total_time] = calculate_missile_obscuration_details(params);
        total_obscuration_time = total_time;
        
    catch ME
        if params(1) > 120 % 只对可能有问题的参数输出调试信息
            fprintf('适应度函数异常: %s\n', ME.message);
        end
        total_obscuration_time = 0;
    end
end

% 计算各导弹的遮蔽时间详情
function [missile_obscuration_times, total_obscuration_time] = calculate_missile_obscuration_details(params)
    global M1_initial_pos M2_initial_pos M3_initial_pos FY1_initial_pos;
    global fake_target_pos true_target_pos missile_speed smoke_sink_speed;
    global smoke_radius smoke_effective_duration g true_target_radius true_target_height;
    global interference_matrix;
    
    try
        % 解包参数
        flight_speed = params(1);
        flight_angle = params(2);
        
        % 提取烟雾弹参数
        num_bombs = sum(sum(interference_matrix > 0, 2) > 0);
        bomb_drop_times = zeros(1, num_bombs);
        bomb_delays = zeros(1, num_bombs);
        
        for i = 1:num_bombs
            drop_time_idx = 2 + (i-1)*2 + 1;
            delay_idx = 2 + (i-1)*2 + 2;
            bomb_drop_times(i) = params(drop_time_idx);
            bomb_delays(i) = params(delay_idx);
        end
        
        % 导弹初始位置
        missile_positions = [M1_initial_pos; M2_initial_pos; M3_initial_pos];
        
        % FY1飞行方向
        fy1_direction = [cos(flight_angle), sin(flight_angle), 0];
        
        % 初始化各导弹的遮蔽时间
        missile_obscuration_times = zeros(1, 3);
        
        % 创建时间轴
        time_step = 0.01;
        max_time = max(bomb_drop_times) + max(bomb_delays) + smoke_effective_duration;
        time_axis = 0:time_step:max_time;
        
        % 对每枚导弹计算遮蔽时间
        for missile_idx = 1:3
            % 导弹轨迹函数
            missile_initial_pos = missile_positions(missile_idx, :);
            missile_direction = (fake_target_pos - missile_initial_pos) / norm(fake_target_pos - missile_initial_pos);
            missile_pos = @(t) missile_initial_pos + missile_direction * missile_speed * t;
            
            % 记录该导弹在各时刻的遮蔽状态
            missile_obscuration_status = false(size(time_axis));
            
            % 检查每枚烟雾弹对该导弹的干扰
            bomb_idx = 0;
            for bomb_row = 1:3
                if sum(interference_matrix(bomb_row, :)) > 0 % 该烟雾弹需要投放
                    bomb_idx = bomb_idx + 1;
                    
                    % 检查该烟雾弹是否干扰当前导弹
                    if interference_matrix(bomb_row, missile_idx) == 1
                        drop_time = bomb_drop_times(bomb_idx);
                        detonation_delay = bomb_delays(bomb_idx);
                        
                        % 计算投放点和起爆点
                        drop_pos = FY1_initial_pos + fy1_direction * flight_speed * drop_time;
                        detonation_time = drop_time + detonation_delay;
                        detonation_pos = drop_pos + fy1_direction * flight_speed * detonation_delay;
                        detonation_pos(3) = detonation_pos(3) - 0.5 * g * detonation_delay^2;
                        
                        % 检查起爆点是否在地面以上
                        if detonation_pos(3) >= 0
                            % 烟幕云团中心位置函数
                            smoke_center_pos = @(t) detonation_pos - [0, 0, smoke_sink_speed * (t - detonation_time)];
                            
                            % 在时间轴上检查遮蔽效果
                            for t_idx = 1:length(time_axis)
                                t = time_axis(t_idx);
                                
                                % 只在烟雾有效时间内检查
                                if t >= detonation_time && t <= (detonation_time + smoke_effective_duration)
                                    current_missile_pos = missile_pos(t);
                                    current_smoke_center = smoke_center_pos(t);
                                    
                                    % 检查烟雾中心是否在地面以上
                                    if current_smoke_center(3) > 0
                                        intersects = cylinder_obscuration_check(current_missile_pos, current_smoke_center, smoke_radius, ...
                                                                               true_target_pos, true_target_radius, true_target_height);
                                        
                                        if intersects
                                            missile_obscuration_status(t_idx) = true;
                                        end
                                    end
                                end
                            end
                        end
                    end
                end
            end
            
            % 计算该导弹的总遮蔽时间
            missile_obscuration_times(missile_idx) = sum(missile_obscuration_status) * time_step;
        end
        
        % 计算总遮蔽时间（所有导弹遮蔽时间的总和）
        total_obscuration_time = sum(missile_obscuration_times);
        
    catch
        missile_obscuration_times = zeros(1, 3);
        total_obscuration_time = 0;
    end
end

% 基于圆柱体目标的遮蔽判断函数（使用theta1和theta2角度比较）
function is_obscured = cylinder_obscuration_check(missile_pos, smoke_center, smoke_radius, cylinder_center, cylinder_radius, cylinder_height)
    % 计算导弹到烟雾球心的向量
    missile_to_smoke = smoke_center(:)' - missile_pos(:)';
    
    % 增加条件验证：导弹到圆柱体中心的向量与烟雾球心到圆柱体中心的向量之积应该大于0
    missile_to_cylinder = cylinder_center(:)' - missile_pos(:)';
    smoke_to_cylinder = cylinder_center(:)' - smoke_center(:)';
    
    % 如果点积小于等于0，说明烟雾球不在导弹和圆柱体之间，不会产生遮蔽
    if dot(missile_to_cylinder, smoke_to_cylinder) <= 0
        is_obscured = false;
        return;
    end
    
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