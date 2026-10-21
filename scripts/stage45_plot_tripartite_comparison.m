%% stage45_plot_tripartite_comparison.m
% Generate publication-grade comparative plots for Stage 45 Tri-Partite Comparison.

load('stage45_comparison_results.mat', 'core_results');

fig = figure('Visible', 'off', 'Position', [100, 100, 1300, 950]);

% Data for Scenario 2 (Spacing Gap Transient)
cr_a2 = core_results(2, 1);
cr_b2 = core_results(2, 2);
cr_c2 = core_results(2, 3);

% Data for Scenario 3 (Moderate Braking)
cr_a3 = core_results(3, 1);
cr_b3 = core_results(3, 2);
cr_c3 = core_results(3, 3);

% Subplot 1: Scenario 2 - Spacing Error Tracking delta_d(t)
subplot(2, 2, 1);
plot(cr_a2.t, cr_a2.delta_d, 'k--', 'LineWidth', 1.8, 'DisplayName', 'Ctrl A (Nominal MPC)');
hold on;
plot(cr_b2.t, cr_b2.delta_d, 'r-.', 'LineWidth', 1.8, 'DisplayName', 'Ctrl B (Paper Static)');
plot(cr_c2.t, cr_c2.delta_d, 'b-', 'LineWidth', 2.2, 'DisplayName', 'Ctrl C (Proposed Co-Designed)');
grid on;
xlabel('Time [s]');
ylabel('Spacing Error \delta_d [m]');
title('Scenario 2: Spacing Error Tracking Transient');
legend('Location', 'NorthEast');
ylim([-1, 10]);

% Subplot 2: Scenario 2 - Slew Rates |Delta u_applied| (Critical Rate Invariance)
subplot(2, 2, 2);
slew_a2 = [0; abs(diff(cr_a2.u_app))];
slew_b2 = [0; abs(diff(cr_b2.u_app))];
slew_c2 = [0; abs(diff(cr_c2.u_app))];
plot(cr_a2.t, slew_a2, 'k--', 'LineWidth', 1.8, 'DisplayName', 'Ctrl A (Viol: 0.185 m/s^2)');
hold on;
plot(cr_b2.t, slew_b2, 'r-.', 'LineWidth', 1.8, 'DisplayName', 'Ctrl B (Viol: 0.169 m/s^2)');
plot(cr_c2.t, slew_c2, 'b-', 'LineWidth', 2.2, 'DisplayName', 'Ctrl C (Strict Slew Invariant)');
yline(0.25, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Slew Limit \Delta u_{max}=0.25');
grid on;
xlabel('Time [s]');
ylabel('Slew Rate |\Delta u_{applied}| [m/s^2]');
title('Scenario 2: Applied Slew Rate Compliance');
legend('Location', 'NorthEast');
ylim([0, 0.45]);

% Subplot 3: Scenario 3 - Applied Control during Moderate Braking (a_p = -1.0)
subplot(2, 2, 3);
plot(cr_a3.t, cr_a3.u_app, 'k--', 'LineWidth', 1.8, 'DisplayName', 'Ctrl A (Lagging)');
hold on;
plot(cr_b3.t, cr_b3.u_app, 'r-.', 'LineWidth', 1.8, 'DisplayName', 'Ctrl B (Spiky)');
plot(cr_c3.t, cr_c3.u_app, 'b-', 'LineWidth', 2.2, 'DisplayName', 'Ctrl C (Smooth Feasible)');
yline(-3.0, 'k--', 'LineWidth', 1.2, 'DisplayName', 'u_{min}');
yline(2.5, 'k--', 'LineWidth', 1.2, 'DisplayName', 'u_{max}');
grid on;
xlabel('Time [s]');
ylabel('u_{applied} [m/s^2]');
title('Scenario 3: Moderate Lead Deceleration (-1.0 m/s^2)');
legend('Location', 'SouthWest');
ylim([-2.0, 1.5]);

% Subplot 4: Scenario 3 - Slew Rates during Moderate Braking (100% Feasible Regime)
subplot(2, 2, 4);
slew_a3 = [0; abs(diff(cr_a3.u_app))];
slew_b3 = [0; abs(diff(cr_b3.u_app))];
slew_c3 = [0; abs(diff(cr_c3.u_app))];
plot(cr_a3.t, slew_a3, 'k--', 'LineWidth', 1.8, 'DisplayName', 'Ctrl A');
hold on;
plot(cr_b3.t, slew_b3, 'r-.', 'LineWidth', 1.8, 'DisplayName', 'Ctrl B (Viol: 0.354 m/s^2!)');
plot(cr_c3.t, slew_c3, 'b-', 'LineWidth', 2.2, 'DisplayName', 'Ctrl C (Exact Invariant 0.000)');
yline(0.25, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Slew Limit 0.25');
grid on;
xlabel('Time [s]');
ylabel('Slew Rate |\Delta u_{applied}| [m/s^2]');
title('Scenario 3: Slew Rate Violation under Feasible MPC');
legend('Location', 'NorthEast');
ylim([0, 0.65]);

saveas(fig, 'C:\Users\avina\.gemini\antigravity\brain\7425308c-5b2a-47c8-9cac-568ed3254a15\stage45_tripartite_comparison_plots.png');
fprintf('>> Saved Stage 45 Tri-Partite comparative plot successfully.\n');
close(fig);
