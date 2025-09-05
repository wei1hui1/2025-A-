% 问题1：计算有效遮蔽时长（修正版）
% 使用导弹视线与烟幕球体相交判断，而非简单的距离判断

clc;
clear;
close all;

% --- 输入参数 ---
% 位置（米）
M1_initial_pos = [20000, 0, 2000];
FY1_initial_pos = [17800, 0, 1800];
fake_target_pos = [0, 0, 0];
true_target_pos = [0, 200, 0];
true_target_radius = 7;
true_target_height = 10;

% 速度（米/秒）
missile_speed = 300;
fy1_speed = 120;
smoke_sink_speed = 3;

% 时间（秒）
drop_time = 1.5;
detonation_delay = 3.6;
smoke_effective_duration = 20;

% 烟幕属性
smoke_radius = 10;

% 其他
g = 9.8; % 重力加速度 m/s^2

% --- 计算 ---

% 1. 导弹M1的运动轨迹（指向假目标）
missile_direction = (fake_target_pos - M1_initial_pos) / norm(fake_target_pos - M1_initial_pos);
missile_pos = @(t) M1_initial_pos + missile_direction * missile_speed * t;

% 2. 无人机FY1的运动轨迹（指向假目标，等高度飞行）
fy1_direction = (fake_target_pos - FY1_initial_pos);
fy1_direction(3) = 0; % 保持高度不变
fy1_direction = fy1_direction / norm(fy1_direction);
fy1_pos = @(t) FY1_initial_pos + fy1_direction * fy1_speed * t;

% 3. 烟幕弹投放点
drop_pos = fy1_pos(drop_time);
drop_velocity = fy1_direction * fy1_speed;

% 4. 烟幕弹起爆点（抛物线运动）
% 烟幕弹在投放时继承了无人机的速度，然后在重力作用下做抛物线运动
detonation_time = drop_time + detonation_delay;
fall_time = detonation_delay;

% 水平方向上，烟幕弹以无人机的速度做匀速直线运动
detonation_pos_x = drop_pos(1) + drop_velocity(1) * fall_time;
detonation_pos_y = drop_pos(2) + drop_velocity(2) * fall_time;

% 垂直方向上，烟幕弹做自由落体运动
detonation_pos_z = drop_pos(3) - 0.5 * g * fall_time^2;
detonation_pos = [detonation_pos_x, detonation_pos_y, detonation_pos_z];

% 5. 烟幕云团中心位置函数（考虑下沉）
smoke_center_pos = @(t) detonation_pos - [0, 0, smoke_sink_speed * (t - detonation_time)];

% 6. 新的遮蔽计算函数：基于圆锥角度判断
function is_obscured = cylinder_obscuration_check(missile_pos, smoke_center, smoke_radius, cylinder_center, cylinder_radius, cylinder_height)
    % 计算导弹是否被烟雾遮蔽圆柱体目标
    % missile_pos: 导弹位置
    % smoke_center: 烟雾球心位置
    % smoke_radius: 烟雾球半径
    % cylinder_center: 圆柱体中心位置
    % cylinder_radius: 圆柱体半径
    % cylinder_height: 圆柱体高度
    
    % 计算导弹到烟雾球心的距离和半锥角theta1
     missile_to_smoke = smoke_center(:)' - missile_pos(:)';
     dist_to_smoke = norm(missile_to_smoke);
    
    % 半锥角theta1：以导弹为顶点，烟雾球为底面的圆锥
    if dist_to_smoke <= smoke_radius
        % 导弹在烟雾球内，完全遮蔽
        is_obscured = true;
        return;
    end
    
    theta1 = asin(smoke_radius / dist_to_smoke);
    
    % 在圆柱表面均匀取点
    num_points_circumference = 36; % 圆周方向36个点
    num_points_height = 10; % 高度方向10个点
    
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
            
            % 如果theta1 <= theta2，说明该点未被遮蔽
            if theta1 <= theta2
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
                
                if theta1 <= theta2
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
                    
                    if theta1 <= theta2
                        is_obscured = false;
                        return;
                    end
                end
            end
        end
    end
end

% 7. 计算有效遮蔽时间
t_start_obscuration = -1;
t_end_obscuration = -1;

% 模拟时间从起爆开始到烟幕失效
time_step = 0.001;
for t = detonation_time : time_step : (detonation_time + smoke_effective_duration)
    
    % 当前时刻的导弹位置
    current_missile_pos = missile_pos(t);
    
    % 当前时刻烟幕云团中心位置
    current_smoke_center = smoke_center_pos(t);
    
    % 使用新的圆柱体遮蔽判断函数
    % 检查烟雾是否遮蔽圆柱体真目标
    is_obscured = cylinder_obscuration_check(current_missile_pos, current_smoke_center, smoke_radius, true_target_pos, true_target_radius, true_target_height);
    
    if is_obscured
        if t_start_obscuration == -1
            t_start_obscuration = t;
        end
        t_end_obscuration = t;
    end
end

% --- 输出结果 ---
if t_start_obscuration ~= -1
    effective_duration = t_end_obscuration - t_start_obscuration;
    fprintf('烟幕弹对M1的有效遮蔽开始时间: %.2f 秒\n', t_start_obscuration);
    fprintf('烟幕弹对M1的有效遮蔽结束时间: %.2f 秒\n', t_end_obscuration);
    fprintf('有效遮蔽总时长: %.2f 秒\n', effective_duration);
    
    % 输出关键位置信息
    fprintf('\n关键位置信息:\n');
    fprintf('烟幕弹投放点: (%.1f, %.1f, %.1f)\n', drop_pos(1), drop_pos(2), drop_pos(3));
    fprintf('烟幕弹起爆点: (%.1f, %.1f, %.1f)\n', detonation_pos(1), detonation_pos(2), detonation_pos(3));
    fprintf('起爆时刻: %.2f 秒\n', detonation_time);
else
    fprintf('在给定的参数下，烟幕弹未能有效遮蔽M1。\n');
end

% --- 可视化 (2D: X-Z平面) ---
figure;
hold on;
grid on;
axis equal;
xlabel('X (m)');
ylabel('Z (m)');
title('问题1：FY1投放烟幕弹对M1的干扰分析（修正版）- X-Z平面视图');

% 绘制目标
plot(fake_target_pos(1), fake_target_pos(3), 'ko', 'MarkerSize', 10, 'MarkerFaceColor', 'k');
text(fake_target_pos(1), fake_target_pos(3) + 100, '假目标', 'FontSize', 10);

% 绘制真目标（矩形表示圆柱体在X-Z平面的投影）
rectangle('Position', [true_target_pos(1)-true_target_radius, true_target_pos(3), 2*true_target_radius, true_target_height], 'FaceColor', 'g', 'FaceAlpha', 0.3);
text(true_target_pos(1), true_target_pos(3) + true_target_height + 50, '真目标', 'FontSize', 10);

% 绘制导弹轨迹
t_sim_end = detonation_time + smoke_effective_duration + 5;
t_points = 0:0.1:t_sim_end;
missile_traj = zeros(length(t_points), 3);
for i = 1:length(t_points)
    missile_traj(i, :) = missile_pos(t_points(i));
end
plot(missile_traj(:,1), missile_traj(:,3), 'r--', 'LineWidth', 1.5);
plot(M1_initial_pos(1), M1_initial_pos(3), 'r^', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
text(M1_initial_pos(1), M1_initial_pos(3), ' M1起点');

% 绘制无人机轨迹
fy1_t_points = 0:0.1:drop_time;
fy1_traj = zeros(length(fy1_t_points), 3);
for i = 1:length(fy1_t_points)
    fy1_traj(i, :) = fy1_pos(fy1_t_points(i));
end
plot(fy1_traj(:,1), fy1_traj(:,3), 'b-', 'LineWidth', 1.5);
plot(FY1_initial_pos(1), FY1_initial_pos(3), 'bo', 'MarkerSize', 10, 'MarkerFaceColor', 'b');
text(FY1_initial_pos(1), FY1_initial_pos(3), ' FY1起点');

% 计算并绘制烟幕弹的平抛运动轨迹
t_trajectory = linspace(0, detonation_delay, 100);
trajectory_x = drop_pos(1) + drop_velocity(1) * t_trajectory;
trajectory_y = drop_pos(2) + drop_velocity(2) * t_trajectory;
trajectory_z = drop_pos(3) - 0.5 * g * t_trajectory.^2;
plot(trajectory_x, trajectory_z, 'g--', 'LineWidth', 2);

% 绘制投放点和起爆点
plot(drop_pos(1), drop_pos(3), 'm*', 'MarkerSize', 8);
text(drop_pos(1), drop_pos(3), ' 投放点');
plot(detonation_pos(1), detonation_pos(3), 'ro', 'MarkerSize', 10, 'MarkerFaceColor', 'r');
text(detonation_pos(1), detonation_pos(3), ' 起爆点');

% 绘制遮蔽期间的关键状态
if t_start_obscuration ~= -1
    % 绘制开始遮蔽时的状态
    t = t_start_obscuration;
    missile_at_start = missile_pos(t);
    smoke_center_at_start = smoke_center_pos(t);
    plot(missile_at_start(1), missile_at_start(3), 'r^', 'MarkerSize', 8);
    plot([missile_at_start(1), true_target_pos(1)], [missile_at_start(3), true_target_pos(3)], 'k:', 'LineWidth', 2);
    
    % 绘制烟幕圆形（球体在X-Z平面的投影）
    theta = 0:0.1:2*pi;
    smoke_x_start = smoke_center_at_start(1) + smoke_radius * cos(theta);
    smoke_z_start = smoke_center_at_start(3) + smoke_radius * sin(theta);
    fill(smoke_x_start, smoke_z_start, 'k', 'FaceAlpha', 0.3, 'EdgeColor', 'k');
    
    % 绘制结束遮蔽时的状态
    t = t_end_obscuration;
    missile_at_end = missile_pos(t);
    smoke_center_at_end = smoke_center_pos(t);
    plot(missile_at_end(1), missile_at_end(3), 'r^', 'MarkerSize', 8);
    plot([missile_at_end(1), true_target_pos(1)], [missile_at_end(3), true_target_pos(3)], 'k:', 'LineWidth', 2);
    smoke_x_end = smoke_center_at_end(1) + smoke_radius * cos(theta);
    smoke_z_end = smoke_center_at_end(3) + smoke_radius * sin(theta);
    fill(smoke_x_end, smoke_z_end, 'k', 'FaceAlpha', 0.3, 'EdgeColor', 'k');
end

legend('假目标', '真目标', 'M1轨迹', 'M1起点', 'FY1轨迹', 'FY1起点', '投放点', '起爆点', 'Location', 'northeast');

% 设置坐标轴范围
xlim([0, 22000]);
ylim([0, 2500]);

fprintf('\n可视化图表已生成，显示了导弹轨迹、无人机轨迹、烟幕位置和遮蔽效果。\n');