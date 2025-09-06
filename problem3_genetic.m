% Q3_new.m - 基于遗传算法的三枚烟幕弹投放策略优化
% 结合problem2_genetic.m的遗传算法和problem2_three_smoke_bombs.m的问题定义

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

fprintf('=== 基于遗传算法的三枚烟幕弹投放策略优化 ===\n\n');
fprintf('导弹到达假目标时间: %.2f秒\n', missile_to_target_time);

% 优化变量：[飞行速度, 飞行角度, 投放时间1, 起爆延迟1, 投放间隔1-2, 起爆延迟2, 投放间隔2-3, 起爆延迟3]
% 约束：投放间隔1-2 >= 1, 投放间隔2-3 >= 1
global_lb = [70, 0, 0, 0.1, 1, 0.1, 1, 0.1];  % 下界
global_ub = [140, 0.175, 5, 5, 3, 5, 3, 5];  % 上界
num_vars = length(global_lb);

fprintf('=== 遗传算法优化开始 ===\n\n');

%% 遗传算法参数设置
tic;
fprintf('遗传算法参数设置...\n');

% 调试开关
global debug_mode;
debug_mode = true;  % 关闭详细调试输出以提高性能

% GA参数
population_size = 30;        % 种群大小
max_generations = 100;       % 最大代数
crossover_rate = 0.8;        % 交叉概率
mutation_rate = 0.1;         % 变异概率
elite_count = 3;             % 精英个体数量
tournament_size = 3;         % 锦标赛选择大小
num_runs = 10;                % 多次运行取最优

% 存储所有运行结果
all_ga_results = [];
best_overall_fitness = -inf;
best_overall_params = [];

% 多次运行遗传算法以提高稳定性
for run = 1:num_runs
    fprintf('\n--- 第 %d 次遗传算法运行 ---\n', run);
    
    % 设置随机种子
    rng(62 + run);
    
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
        if mod(generation, 20) == 0 || generation == 1
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
        
        % 早停条件（连续30代无改善）
        if generation > 30
            recent_best = max(fitness_history(max(1, generation-29):generation, 1));
            if recent_best - fitness_history(max(1, generation-29), 1) < 1e-6
                fprintf('早停：连续30代无显著改善\n');
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
fprintf('遗传算法最佳参数: [%.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', best_overall_params);
fprintf('遗传算法最佳方向角: %.2f度\n', best_overall_params(2) * 180 / pi);

% 显示多次运行的统计信息
fprintf('\n=== 遗传算法多次运行统计 ===\n');
fprintf('运行次数: %d\n', num_runs);
fprintf('适应度范围: %.6f - %.6f\n', min(all_ga_results(:,1)), max(all_ga_results(:,1)));
fprintf('适应度标准差: %.6f\n', std(all_ga_results(:,1)));
fprintf('方向角范围: %.2f° - %.2f°\n', min(all_ga_results(:,3)) * 180 / pi, max(all_ga_results(:,3)) * 180 / pi);
fprintf('方向角标准差: %.2f°\n', std(all_ga_results(:,3)) * 180 / pi);

%% 输出最终结果
final_result = best_overall_params;
final_fitness = best_overall_fitness;

% 计算实际投放时间用于显示
drop_time1 = final_result(3);
drop_time2 = drop_time1 + final_result(5);
drop_time3 = drop_time2 + final_result(7);

fprintf('\n=== 三枚烟幕弹投放策略结果 ===\n');
fprintf('最优飞行速度: %.3f m/s\n', final_result(1));
fprintf('最优飞行角度: %.3f 度\n', final_result(2) * 180 / pi);
fprintf('第1枚弹投放时间: %.3f 秒\n', drop_time1);
fprintf('第1枚弹起爆延迟: %.3f 秒\n', final_result(4));
fprintf('第2枚弹投放时间: %.3f 秒\n', drop_time2);
fprintf('第2枚弹起爆延迟: %.3f 秒\n', final_result(6));
fprintf('第3枚弹投放时间: %.3f 秒\n', drop_time3);
fprintf('第3枚弹起爆延迟: %.3f 秒\n', final_result(8));
fprintf('总遮蔽时长: %.5f 秒\n', final_fitness);

% 验证时间间隔
fprintf('\n=== 时间间隔验证 ===\n');
fprintf('第1枚与第2枚弹投放间隔: %.3f 秒\n', final_result(5));
fprintf('第2枚与第3枚弹投放间隔: %.3f 秒\n', final_result(7));

% 计算各枚弹的详细信息和遮蔽贡献
opt_flight_speed = final_result(1);
opt_flight_angle = final_result(2);
opt_fy1_direction = [cos(opt_flight_angle), sin(opt_flight_angle), 0];

% 计算实际投放时间
actual_drop_times = [drop_time1, drop_time2, drop_time3];
detonation_delays = [final_result(4), final_result(6), final_result(8)];

% 计算每颗烟雾弹的单独遮蔽贡献
[individual_contributions, combined_contribution, obscuration_intervals] = calculate_individual_obscuration_contributions(final_result);

fprintf('\n=== 各枚烟雾弹遮蔽贡献分析 ===\n');
for bomb_idx = 1:3
    drop_time = actual_drop_times(bomb_idx);
    detonation_delay = detonation_delays(bomb_idx);
    
    drop_pos = FY1_initial_pos + opt_fy1_direction * opt_flight_speed * drop_time;
    detonation_time = drop_time + detonation_delay;
    detonation_pos = drop_pos + opt_fy1_direction * opt_flight_speed * detonation_delay;
    detonation_pos(3) = detonation_pos(3) - 0.5 * g * detonation_delay^2;
    
    fprintf('\n第%d枚弹详细信息:\n', bomb_idx);
    fprintf('  投放点: (%.3f, %.3f, %.3f)\n', drop_pos(1), drop_pos(2), drop_pos(3));
    fprintf('  起爆点: (%.3f, %.3f, %.3f)\n', detonation_pos(1), detonation_pos(2), detonation_pos(3));
    fprintf('  起爆时刻: %.3f 秒\n', detonation_time);
    fprintf('  单独遮蔽贡献: %.5f 秒\n', individual_contributions(bomb_idx));
    if combined_contribution > 0
        fprintf('  遮蔽贡献占比: %.2f%%\n', individual_contributions(bomb_idx) / combined_contribution * 100);
    else
        fprintf('  遮蔽贡献占比: N/A (总遮蔽时间为0)\n');
    end
    
    % 打印遮蔽时间区间
    intervals = obscuration_intervals{bomb_idx};
    if isempty(intervals)
        fprintf('  遮蔽时间区间: 无\n');
    else
        fprintf('  遮蔽时间区间: ');
        for i = 1:size(intervals, 1)
            if i > 1
                fprintf(', ');
            end
            fprintf('[%.3f, %.3f]', intervals(i, 1), intervals(i, 2));
        end
        fprintf(' 秒\n');
    end
    
    % 添加烟雾弹有效性分析
    if individual_contributions(bomb_idx) == 0
        if detonation_pos(3) < 0
            fprintf('  ⚠️  起爆点在地面以下，烟雾弹无效\n');
        else
            fprintf('  ⚠️  起爆点在地面以上但无遮蔽效果，可能时机不当\n');
        end
    else
        fprintf('  ✓  烟雾弹有效，产生遮蔽效果\n');
    end
end

fprintf('\n=== 遮蔽效果汇总 ===\n');
fprintf('各弹单独遮蔽时间总和: %.5f 秒\n', sum(individual_contributions));
fprintf('三弹联合遮蔽时间: %.5f 秒\n', combined_contribution);
fprintf('协同效应损失: %.5f 秒 (%.2f%%)\n', sum(individual_contributions) - combined_contribution, ...
        (sum(individual_contributions) - combined_contribution) / sum(individual_contributions) * 100);

fprintf('\n三枚烟幕弹投放策略优化完成！\n');
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

% 计算每颗烟雾弹的单独遮蔽贡献
function [individual_contributions, combined_contribution, obscuration_intervals] = calculate_individual_obscuration_contributions(params)
    global M1_initial_pos FY1_initial_pos fake_target_pos true_target_pos;
    global missile_speed smoke_sink_speed smoke_radius smoke_effective_duration g;
    global true_target_radius true_target_height;
    
    % 解包参数
    flight_speed = params(1);
    flight_angle = params(2);
    drop_time1 = params(3);
    detonation_delay1 = params(4);
    interval1_2 = params(5);
    detonation_delay2 = params(6);
    interval2_3 = params(7);
    detonation_delay3 = params(8);
    
    % 计算实际投放时间
    drop_time2 = drop_time1 + interval1_2;
    drop_time3 = drop_time2 + interval2_3;
    
    % 导弹轨迹函数
    missile_direction = (fake_target_pos - M1_initial_pos) / norm(fake_target_pos - M1_initial_pos);
    missile_pos = @(t) M1_initial_pos + missile_direction * missile_speed * t;
    
    % FY1飞行方向
    fy1_direction = [cos(flight_angle), sin(flight_angle), 0];
    
    % 创建时间轴
    time_step = 0.01;
    max_time = max([drop_time1, drop_time2, drop_time3]) + max([detonation_delay1, detonation_delay2, detonation_delay3]) + smoke_effective_duration;
    time_axis = 0:time_step:max_time;
    
    % 计算每颗烟雾弹的单独遮蔽贡献
    individual_contributions = zeros(1, 3);
    obscuration_intervals = cell(1, 3);  % 存储每个烟雾弹的遮蔽时间区间
    drop_times = [drop_time1, drop_time2, drop_time3];
    detonation_delays = [detonation_delay1, detonation_delay2, detonation_delay3];
    
    for bomb_idx = 1:3
        drop_time = drop_times(bomb_idx);
        detonation_delay = detonation_delays(bomb_idx);
        
        % 投放点和起爆点
        drop_pos = FY1_initial_pos + fy1_direction * flight_speed * drop_time;
        detonation_time = drop_time + detonation_delay;
        detonation_pos = drop_pos + fy1_direction * flight_speed * detonation_delay;
        detonation_pos(3) = detonation_pos(3) - 0.5 * g * detonation_delay^2;
        
        % 初始化遮蔽区间
        obscuration_intervals{bomb_idx} = [];
        
        % 检查起爆点是否在地面以上
        if detonation_pos(3) < 0
            individual_contributions(bomb_idx) = 0;
            continue;
        end
        
        % 烟幕云团中心位置
        smoke_center_pos = @(t) detonation_pos - [0, 0, smoke_sink_speed * (t - detonation_time)];
        
        % 计算该烟雾弹的单独遮蔽时间
        obscuration_status = false(size(time_axis));
        
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
                        obscuration_status(t_idx) = true;
                    end
                end
            end
        end
        
        % 计算该烟雾弹的遮蔽时间
        individual_contributions(bomb_idx) = sum(obscuration_status) * time_step;
        
        % 提取遮蔽时间区间
        intervals = [];
        in_interval = false;
        interval_start = 0;
        
        for t_idx = 1:length(time_axis)
            if obscuration_status(t_idx) && ~in_interval
                % 开始新的遮蔽区间
                in_interval = true;
                interval_start = time_axis(t_idx);
            elseif ~obscuration_status(t_idx) && in_interval
                % 结束当前遮蔽区间
                in_interval = false;
                interval_end = time_axis(t_idx - 1);
                intervals = [intervals; interval_start, interval_end];
            end
        end
        
        % 处理最后一个区间（如果在时间轴结束时仍在遮蔽状态）
        if in_interval
            interval_end = time_axis(end);
            intervals = [intervals; interval_start, interval_end];
        end
        
        obscuration_intervals{bomb_idx} = intervals;
    end
    
    % 计算联合遮蔽时间（已有函数）
    combined_contribution = calculate_combined_obscuration_time(params);
end

% 评估种群适应度
function fitness = evaluate_population(population)
    pop_size = size(population, 1);
    fitness = zeros(pop_size, 1);
    
    for i = 1:pop_size
        fitness(i) = fitness_function_three_bombs(population(i, :));
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

% 三枚烟幕弹的适应度函数
function total_duration = fitness_function_three_bombs(params)
    % 解包参数
    flight_speed = params(1);
    flight_angle = params(2);
    drop_time1 = params(3);
    detonation_delay1 = params(4);
    interval1_2 = params(5);
    detonation_delay2 = params(6);
    interval2_3 = params(7);
    detonation_delay3 = params(8);
    
    % 参数边界检查
    global_lb = [70, 0, 0, 0.1, 1, 0.1, 1, 0.1];
    global_ub = [140, 0.175, 5, 5, 3, 5, 3, 5];
    
    % 检查参数是否在边界内
    for i = 1:length(params)
        if params(i) < global_lb(i) || params(i) > global_ub(i)
            total_duration = 0;
            return;
        end
    end
    
    % 计算实际投放时间
    drop_times = [drop_time1, drop_time1 + interval1_2, drop_time1 + interval1_2 + interval2_3];
    detonation_delays = [detonation_delay1, detonation_delay2, detonation_delay3];
    
    % 全局变量访问
    global M1_initial_pos FY1_initial_pos fake_target_pos true_target_pos;
    global missile_speed smoke_sink_speed smoke_radius smoke_effective_duration g;
    global true_target_radius true_target_height;
    
    try
        % 导弹轨迹
        missile_direction = (fake_target_pos - M1_initial_pos) / norm(fake_target_pos - M1_initial_pos);
        missile_pos = @(t) M1_initial_pos + missile_direction * missile_speed * t;
        
        % 无人机轨迹
        fy1_direction = [cos(flight_angle), sin(flight_angle), 0];
        
        % 计算联合遮蔽时间（正确处理重叠效应）
        total_duration = calculate_combined_obscuration_time(params);
        
    catch ME
        % 输出错误信息用于调试
        if flight_speed > 120  % 只对可能有问题的参数输出调试信息
            fprintf('适应度函数异常: %s, 参数: [%.2f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f]\n', ...
                    ME.message, params);
        end
        total_duration = 0;
    end
end

% 计算多个烟雾弹联合遮蔽时间的函数
function total_time = calculate_combined_obscuration_time(params)
    global M1_initial_pos FY1_initial_pos fake_target_pos true_target_pos;
    global missile_speed smoke_sink_speed smoke_radius smoke_effective_duration g;
    global true_target_radius true_target_height;
    
    try
        % 解包参数
        flight_speed = params(1);
        flight_angle = params(2);
        drop_time1 = params(3);
        detonation_delay1 = params(4);
        interval1_2 = params(5);
        detonation_delay2 = params(6);
        interval2_3 = params(7);
        detonation_delay3 = params(8);
        
        % 计算实际投放时间
        drop_time2 = drop_time1 + interval1_2;
        drop_time3 = drop_time2 + interval2_3;
        
        % 导弹轨迹函数
        missile_direction = (fake_target_pos - M1_initial_pos) / norm(fake_target_pos - M1_initial_pos);
        missile_pos = @(t) M1_initial_pos + missile_direction * missile_speed * t;
        
        % FY1飞行方向
        fy1_direction = [cos(flight_angle), sin(flight_angle), 0];
        
        % 创建时间轴数组，用于记录每个时刻的遮蔽状态
        time_step = 0.01;
        max_time = max([drop_time1, drop_time2, drop_time3]) + max([detonation_delay1, detonation_delay2, detonation_delay3]) + smoke_effective_duration;
        time_axis = 0:time_step:max_time;
        obscuration_status = false(size(time_axis));
        
        % 计算每枚烟雾弹的遮蔽时间段
        drop_times = [drop_time1, drop_time2, drop_time3];
        detonation_delays = [detonation_delay1, detonation_delay2, detonation_delay3];
        
        for bomb_idx = 1:3
            drop_time = drop_times(bomb_idx);
            detonation_delay = detonation_delays(bomb_idx);
            
            % 投放点和起爆点
            drop_pos = FY1_initial_pos + fy1_direction * flight_speed * drop_time;
            detonation_time = drop_time + detonation_delay;
            detonation_pos = drop_pos + fy1_direction * flight_speed * detonation_delay;
            detonation_pos(3) = detonation_pos(3) - 0.5 * g * detonation_delay^2;
            
            % 检查起爆点是否在地面以上
            if detonation_pos(3) < 0
                continue;
            end
            
            % 烟幕云团中心位置
            smoke_center_pos = @(t) detonation_pos - [0, 0, smoke_sink_speed * (t - detonation_time)];
            
            % 在时间轴上标记该烟雾弹的遮蔽时间段
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
                            obscuration_status(t_idx) = true;
                        end
                    end
                end
            end
        end
        
        % 计算总遮蔽时间（连续遮蔽时间段的总和）
        total_time = sum(obscuration_status) * time_step;
        
    catch
        total_time = 0;
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