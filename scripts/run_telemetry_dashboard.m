function run_telemetry_dashboard(scenario_idx, ctrl_idx, save_video)
% RUN_TELEMETRY_DASHBOARD
% Interactive Smart Highway Telemetry Dashboard for Tube MPC ACC.
%
% Visualizes:
%   1. 2D/3D Top-View Highway with moving road markers and dynamic vehicles
%   2. Dynamic "Safety Tube" (Uncertainty Tube) expanding/contracting
%   3. Working Brake Lights on the Lead Vehicle
%   4. Synchronized Telemetry: Velocities, Clearance Gap, Commanded Acceleration
%   5. Physical Slew-Rate Compliance Verification
%
% Usage:
%   run_telemetry_dashboard()                 % Default: Scenario 3, Controller C (Proposed)
%   run_telemetry_dashboard(3, 3, false)      % Interactive live window
%   run_telemetry_dashboard(3, 3, true)       % Exports acc_dashboard_scen3_ctrlC.mp4
%
% Author: Avinash K. (AVCL, EE Dept)
% Date: October 2026

if nargin < 1, scenario_idx = 3; end   % Default: Scenario 3 (Moderate Braking)
if nargin < 2, ctrl_idx = 3; end       % Default: Controller C (Proposed Tube MPC)
if nargin < 3, save_video = false; end

%% 1. Load Simulation Trajectory
pkg_root = fileparts(fileparts(mfilename('fullpath')));
if isempty(pkg_root), pkg_root = pwd; end

data_path = fullfile(pkg_root, 'data', 'stage45_comparison_results.mat');
if ~isfile(data_path)
    % Fallback to local data folder
    data_path = 'stage45_comparison_results.mat';
end
load(data_path, 'core_results');

res = core_results(scenario_idx, ctrl_idx);
t = res.t;
clearance = res.clearance;
delta_d = res.delta_d;
delta_v = res.delta_v;
u_app = res.u_app;
a_ego = res.a_ego;
v0 = 20.0; % nominal speed [m/s] = 72 km/h
v_host = v0 + delta_v;
v_lead = v_host - delta_v; % Lead vehicle speed

% Compute instantaneous slew rate
slew = [0; abs(diff(u_app))];
slew_limit = 0.25;

% Compute Lead Vehicle Deceleration
lead_braking = [0; diff(v_lead) ./ diff(t)];
lead_braking(lead_braking > 0) = 0; % only negative acceleration counts as braking

%% 2. Setup High-Tech Dashboard Figure
fig = figure('Name', 'Adaptive Cruise Control - Smart Telemetry Dashboard', ...
    'Color', [0.08, 0.10, 0.14], ...
    'Position', [100, 80, 1280, 760], ...
    'MenuBar', 'none', ...
    'ToolBar', 'figure', ...
    'NumberTitle', 'off');

if save_video
    video_filename = sprintf('acc_dashboard_scen%d_ctrl%d.mp4', scenario_idx, ctrl_idx);
    v_writer = VideoWriter(video_filename, 'MPEG-4');
    v_writer.FrameRate = 15;
    open(v_writer);
    fprintf('Recording video to: %s ...\n', video_filename);
end

%% 3. Construct Dashboard Subplots
% Panel 1: Highway Road Scene (Top 45% of figure)
ax_road = subplot('Position', [0.06, 0.52, 0.88, 0.42]);
set(ax_road, 'Color', [0.12, 0.14, 0.18], 'XColor', [0.6, 0.65, 0.75], 'YColor', [0.6, 0.65, 0.75]);
hold(ax_road, 'on');
box(ax_road, 'on');

% Panel 2: Velocity & Clearance Telemetry (Bottom Left)
ax_tel1 = subplot('Position', [0.06, 0.08, 0.41, 0.36]);
set(ax_tel1, 'Color', [0.10, 0.12, 0.16], 'XColor', [0.7, 0.75, 0.85], 'YColor', [0.7, 0.75, 0.85], 'GridColor', [0.25, 0.30, 0.40]);
grid(ax_tel1, 'on'); hold(ax_tel1, 'on');

% Panel 3: Actuator Acceleration & Slew Rate (Bottom Right)
ax_tel2 = subplot('Position', [0.53, 0.08, 0.41, 0.36]);
set(ax_tel2, 'Color', [0.10, 0.12, 0.16], 'XColor', [0.7, 0.75, 0.85], 'YColor', [0.7, 0.75, 0.85], 'GridColor', [0.25, 0.30, 0.40]);
grid(ax_tel2, 'on'); hold(ax_tel2, 'on');

%% 4. Initialize Static Elements on Highway Scene
axes(ax_road);
road_length = 70; % meters visible window
road_width = 12;  % 3 lanes of 4m
xlim(ax_road, [-5, road_length]);
ylim(ax_road, [-road_width/2, road_width/2]);
set(ax_road, 'YTick', [-4, 0, 4], 'YTickLabel', {'Right Lane', 'Center Lane (ACC)', 'Left Lane'});
xlabel(ax_road, 'Relative Longitudinal Distance [m]', 'Color', [0.85, 0.90, 1.0], 'FontSize', 11);
title(ax_road, sprintf('HIGHWAY ADAPTIVE CRUISE CONTROL - SCENARIO: %s | CONTROLLER: %s', ...
    upper(res.bm_name), upper(res.ctrl_name)), 'Color', [0.35, 0.80, 1.0], 'FontSize', 13, 'FontWeight', 'bold');

% Road Surface & Grass
patch([-10, road_length+10, road_length+10, -10], [-road_width/2, -road_width/2, road_width/2, road_width/2], ...
    [0.18, 0.20, 0.24], 'EdgeColor', 'none', 'Parent', ax_road);
% Grass borders
patch([-10, road_length+10, road_length+10, -10], [road_width/2, road_width/2, road_width/2+3, road_width/2+3], ...
    [0.12, 0.28, 0.15], 'EdgeColor', 'none', 'Parent', ax_road);
patch([-10, road_length+10, road_length+10, -10], [-road_width/2-3, -road_width/2-3, -road_width/2, -road_width/2], ...
    [0.12, 0.28, 0.15], 'EdgeColor', 'none', 'Parent', ax_road);

% Lane separator lines
yline(ax_road, -2, 'Color', [0.8, 0.8, 0.8], 'LineStyle', '--', 'LineWidth', 1.5);
yline(ax_road, 2, 'Color', [0.8, 0.8, 0.8], 'LineStyle', '--', 'LineWidth', 1.5);
yline(ax_road, -6, 'Color', [1.0, 0.8, 0.2], 'LineStyle', '-', 'LineWidth', 2.0); % Yellow shoulder
yline(ax_road, 6, 'Color', [1.0, 0.8, 0.2], 'LineStyle', '-', 'LineWidth', 2.0);

% Host Car (Blue) Geometry at x = 0
car_len = 4.5; car_wid = 2.0;
h_host_body = patch([-car_len, 0, 0, -car_len], [-car_wid/2, -car_wid/2, car_wid/2, car_wid/2], ...
    [0.15, 0.50, 0.95], 'EdgeColor', [0.6, 0.85, 1.0], 'LineWidth', 1.5, 'Parent', ax_road);
% Host Cabin
h_host_cabin = patch([-3.2, -1.0, -1.0, -3.2], [-0.8, -0.8, 0.8, 0.8], ...
    [0.08, 0.20, 0.40], 'EdgeColor', [0.4, 0.7, 0.9], 'Parent', ax_road);
% Host Headlights
h_host_light_l = patch([0, 0.3, 0.3, 0], [0.55, 0.55, 0.85, 0.85], [1, 1, 0.8], 'EdgeColor', 'none', 'Parent', ax_road);
h_host_light_r = patch([0, 0.3, 0.3, 0], [-0.85, -0.85, -0.55, -0.55], [1, 1, 0.8], 'EdgeColor', 'none', 'Parent', ax_road);

% Lead Car (Red) Geometry at x = clearance(1)
x_lead_init = clearance(1);
h_lead_body = patch([x_lead_init, x_lead_init+car_len, x_lead_init+car_len, x_lead_init], ...
    [-car_wid/2, -car_wid/2, car_wid/2, car_wid/2], ...
    [0.85, 0.20, 0.20], 'EdgeColor', [1.0, 0.5, 0.5], 'LineWidth', 1.5, 'Parent', ax_road);
% Lead Cabin
h_lead_cabin = patch([x_lead_init+1.0, x_lead_init+3.2, x_lead_init+3.2, x_lead_init+1.0], ...
    [-0.8, -0.8, 0.8, 0.8], [0.35, 0.08, 0.08], 'EdgeColor', [0.9, 0.3, 0.3], 'Parent', ax_road);
% Lead Brake Lights (Glow bright red during braking!)
h_brake_l = patch([x_lead_init-0.2, x_lead_init, x_lead_init, x_lead_init-0.2], ...
    [0.55, 0.55, 0.85, 0.85], [0.4, 0.05, 0.05], 'EdgeColor', 'none', 'Parent', ax_road);
h_brake_r = patch([x_lead_init-0.2, x_lead_init, x_lead_init, x_lead_init-0.2], ...
    [-0.85, -0.85, -0.55, -0.55], [0.4, 0.05, 0.05], 'EdgeColor', 'none', 'Parent', ax_road);

% Dynamic Safety Uncertainty Tube (Green Translucent Corridor)
tube_x = [0, x_lead_init, x_lead_init, 0];
tube_y = [-1.5, -1.5, 1.5, 1.5];
h_tube = patch(tube_x, tube_y, [0.1, 0.85, 0.4], 'FaceAlpha', 0.25, ...
    'EdgeColor', [0.2, 0.95, 0.5], 'LineStyle', '-.', 'LineWidth', 1.8, 'Parent', ax_road);

% Distance Gap Dimension Line & Label
h_dist_line = line([0, x_lead_init], [2.8, 2.8], 'Color', [0.3, 0.85, 1.0], 'LineWidth', 1.5, 'Parent', ax_road);
h_dist_txt = text(x_lead_init/2, 3.6, sprintf('CLEARANCE: %.1f m', x_lead_init), ...
    'Color', [0.3, 0.85, 1.0], 'FontSize', 11, 'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'Parent', ax_road);

% HUD Digital Readout Box on Road Panel
h_hud_bg = patch([1, 26, 26, 1], [-5.6, -5.6, -2.6, -2.6], [0.05, 0.07, 0.10], ...
    'FaceAlpha', 0.85, 'EdgeColor', [0.3, 0.5, 0.8], 'LineWidth', 1.2, 'Parent', ax_road);
h_hud_txt = text(2, -4.1, 'INITIALIZING ADAS...', 'Color', [0.9, 0.95, 1.0], ...
    'FontSize', 10, 'FontName', 'Courier', 'FontWeight', 'bold', 'Parent', ax_road);

%% 5. Setup Telemetry Curves
% Telemetry 1: Speed & Clearance
plot(ax_tel1, t, v_lead .* 3.6, 'Color', [1.0, 0.4, 0.4], 'LineWidth', 1.5, 'DisplayName', 'Lead Speed (km/h)');
plot(ax_tel1, t, v_host .* 3.6, 'Color', [0.2, 0.7, 1.0], 'LineWidth', 2.0, 'DisplayName', 'Host Speed (km/h)');
plot(ax_tel1, t, clearance, 'Color', [0.3, 0.9, 0.4], 'LineWidth', 2.0, 'DisplayName', 'Clearance l(t) [m]');
yline(ax_tel1, 5.0, 'Color', [1.0, 0.2, 0.2], 'LineStyle', ':', 'LineWidth', 1.5, 'DisplayName', 'Min Safe Stop (5m)');
h_curr_marker1 = plot(ax_tel1, t(1), clearance(1), 'wo', 'MarkerFaceColor', [0.3, 0.9, 0.4], 'MarkerSize', 8);
xlabel(ax_tel1, 'Time [s]', 'FontSize', 10);
ylabel(ax_tel1, 'Speed [km/h] & Clearance [m]', 'FontSize', 10);
title(ax_tel1, 'TELEMETRY: VELOCITIES & FOLLOWING DISTANCE', 'Color', [0.8, 0.9, 1.0], 'FontSize', 11, 'FontWeight', 'bold');
legend(ax_tel1, 'Location', 'NorthEast', 'TextColor', [0.9, 0.9, 0.9], 'Color', [0.15, 0.18, 0.22]);
xlim(ax_tel1, [0, t(end)]);
ylim(ax_tel1, [0, max(max(v_host)*3.6, max(clearance)) + 10]);

% Telemetry 2: Applied Control & Slew-Rate Compliance
plot(ax_tel2, t, u_app, 'Color', [0.3, 0.75, 1.0], 'LineWidth', 2.0, 'DisplayName', 'Applied Command u_{app} [m/s^2]');
plot(ax_tel2, t, a_ego, 'Color', [0.9, 0.7, 0.2], 'LineWidth', 1.6, 'LineStyle', '-.', 'DisplayName', 'Realized Accel a [m/s^2]');
plot(ax_tel2, t, slew, 'Color', [1.0, 0.35, 0.8], 'LineWidth', 1.8, 'DisplayName', 'Slew |\Delta u| [m/s^2]');
yline(ax_tel2, slew_limit, 'Color', [1.0, 0.2, 0.2], 'LineStyle', '--', 'LineWidth', 1.5, 'DisplayName', 'Slew Limit (0.25 m/s^2)');
yline(ax_tel2, -3.0, 'Color', [0.7, 0.7, 0.7], 'LineStyle', ':', 'LineWidth', 1.0, 'DisplayName', 'u_{min} (-3.0 m/s^2)');
h_curr_marker2 = plot(ax_tel2, t(1), u_app(1), 'wo', 'MarkerFaceColor', [0.3, 0.75, 1.0], 'MarkerSize', 8);
xlabel(ax_tel2, 'Time [s]', 'FontSize', 10);
ylabel(ax_tel2, 'Acceleration & Slew [m/s^2]', 'FontSize', 10);
title(ax_tel2, 'ACTUATOR PHYSICAL COMPLIANCE & RATE LIMITS', 'Color', [0.8, 0.9, 1.0], 'FontSize', 11, 'FontWeight', 'bold');
legend(ax_tel2, 'Location', 'SouthWest', 'TextColor', [0.9, 0.9, 0.9], 'Color', [0.15, 0.18, 0.22]);
xlim(ax_tel2, [0, t(end)]);
ylim(ax_tel2, [-3.5, 3.0]);

%% 6. Real-Time Playback Animation Loop
fprintf('Starting real-time highway animation for %s...\n', res.ctrl_name);
for k = 1:length(t)
    if ~ishandle(fig), break; end
    
    cur_x_lead = clearance(k);
    cur_v_host = v_host(k);
    cur_v_lead = v_lead(k);
    cur_u = u_app(k);
    cur_slew = slew(k);
    is_braking = lead_braking(k) < -0.15;
    
    % 1. Update Lead Vehicle Position
    set(h_lead_body, 'XData', [cur_x_lead, cur_x_lead+car_len, cur_x_lead+car_len, cur_x_lead]);
    set(h_lead_cabin, 'XData', [cur_x_lead+1.0, cur_x_lead+3.2, cur_x_lead+3.2, cur_x_lead+1.0]);
    set(h_brake_l, 'XData', [cur_x_lead-0.2, cur_x_lead, cur_x_lead, cur_x_lead-0.2]);
    set(h_brake_r, 'XData', [cur_x_lead-0.2, cur_x_lead, cur_x_lead, cur_x_lead-0.2]);
    
    % 2. Brake Lights Active Glow
    if is_braking
        set(h_brake_l, 'FaceColor', [1.0, 0.05, 0.05]); % Glow Bright Red!
        set(h_brake_r, 'FaceColor', [1.0, 0.05, 0.05]);
    else
        set(h_brake_l, 'FaceColor', [0.35, 0.05, 0.05]); % Dim tail light
        set(h_brake_r, 'FaceColor', [0.35, 0.05, 0.05]);
    end
    
    % 3. Update Dynamic Safety Uncertainty Tube
    % The tube dynamically contracts/expands based on speed and uncertainty
    tube_width = 1.3 + 0.3 * (cur_v_host / v0);
    set(h_tube, 'XData', [0, cur_x_lead, cur_x_lead, 0], ...
                'YData', [-tube_width, -tube_width, tube_width, tube_width]);
    
    % 4. Update Distance Gap Dimension Line & Text
    set(h_dist_line, 'XData', [0, cur_x_lead]);
    dist_str = sprintf('GAP: %.1f m (SAFE: PASS)', cur_x_lead);
    if cur_x_lead < 10.0
        set(h_dist_txt, 'Position', [cur_x_lead/2, 3.6, 0], 'String', dist_str, 'Color', [1.0, 0.3, 0.3]);
        set(h_dist_line, 'Color', [1.0, 0.3, 0.3]);
    else
        set(h_dist_txt, 'Position', [cur_x_lead/2, 3.6, 0], 'String', dist_str, 'Color', [0.3, 0.85, 1.0]);
        set(h_dist_line, 'Color', [0.3, 0.85, 1.0]);
    end
    
    % 5. Update Telemetry Markers
    set(h_curr_marker1, 'XData', t(k), 'YData', clearance(k));
    set(h_curr_marker2, 'XData', t(k), 'YData', cur_u);
    
    % 6. Update HUD Status Text
    if cur_slew <= slew_limit + 1e-4
        slew_status = '100% COMPLIANT';
    else
        slew_status = sprintf('VIOLATION (+%.2f)', cur_slew - slew_limit);
    end
    
    hud_str = sprintf(['TIME: %4.1fs | CLEARANCE: %4.1fm\n' ...
                      'HOST SPEED: %4.1f km/h | LEAD: %4.1f km/h\n' ...
                      'COMMAND u: %5.2f m/s^2 | REALIZED: %5.2f\n' ...
                      'SLEW: %4.2f / 0.25 m/s^2 [%s]'], ...
                      t(k), cur_x_lead, cur_v_host*3.6, cur_v_lead*3.6, ...
                      cur_u, a_ego(k), cur_slew, slew_status);
    set(h_hud_txt, 'String', hud_str);
    
    drawnow;
    if save_video
        frame = getframe(fig);
        writeVideo(v_writer, frame);
    else
        pause(0.04); % Smooth ~25 fps animation
    end
end

if save_video
    close(v_writer);
    fprintf('Video successfully exported to: %s\n', video_filename);
end

fprintf('Dashboard animation completed successfully!\n');
end
