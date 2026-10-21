% STAGE46_PLOT_ABLATION.M
% Generates publication-quality figure visualizing the 5-variant ablation study.

clear; clc;
load('stage46_ablation_results.mat');

fig = figure('Color', 'k', 'Position', [100, 100, 1200, 800], 'Visible', 'off');

% Colors for 5 variants
colors = {
    [0.7, 0.7, 0.7], ... % Var-A: Gray dashed
    [1.0, 0.6, 0.0], ... % Var-B: Orange dashed
    [0.9, 0.2, 0.2], ... % Var-C: Red solid
    [0.2, 0.8, 0.8], ... % Var-D: Cyan solid
    [0.0, 0.5, 1.0]  ... % Var-E: Bright Blue solid
};
lines = {'--', '--', '-', '-', '-'};
widths = {1.5, 1.5, 1.8, 1.8, 2.5};

var_labels = {
    'Var-A: Static, NoStep0, K_{paper}', ...
    'Var-B: Traj, NoStep0, K_{paper}', ...
    'Var-C: Static, Step0, K_{paper}', ...
    'Var-D: Traj, Step0, K_{paper}', ...
    'Var-E: Traj, Step0, K_{opt} (Proposed)'
};

% Panel 1: Core-3 Slew Rate Comparison (The Slew Violation Proof)
subplot(2, 2, 1);
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
hold on; grid on;
yline(0.25, 'w--', 'LineWidth', 1.5, 'DisplayName', 'Slew Limit \Delta u_{max}=0.25');
for v = 1:5
    res = ablation_results(3, v);
    plot(res.t, abs(res.delta_u_applied), 'Color', colors{v}, 'LineStyle', lines{v}, ...
        'LineWidth', widths{v}, 'DisplayName', var_labels{v});
end
title('Core-3: Applied Slew Rate Compliance |\Delta u_{app}|', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time [s]', 'Color', 'w'); ylabel('|\Delta u_{app}| [m/s^2]', 'Color', 'w');
ylim([0, 0.65]); xlim([0, 5]);
legend('Location', 'northeast', 'TextColor', 'w', 'Color', [0.1 0.1 0.1], 'FontSize', 8);

% Panel 2: Core-3 Applied Acceleration u_app
subplot(2, 2, 2);
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
hold on; grid on;
for v = 1:5
    res = ablation_results(3, v);
    plot(res.t, res.u_applied, 'Color', colors{v}, 'LineStyle', lines{v}, ...
        'LineWidth', widths{v}, 'DisplayName', var_labels{v});
end
title('Core-3: Applied Acceleration u_{app} (Control Smoothness)', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time [s]', 'Color', 'w'); ylabel('u_{app} [m/s^2]', 'Color', 'w');
ylim([-2.5, 1.8]); xlim([0, 5]);
legend('Location', 'southwest', 'TextColor', 'w', 'Color', [0.1 0.1 0.1], 'FontSize', 8);

% Panel 3: Core-2 Spacing Error Transient
subplot(2, 2, 3);
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
hold on; grid on;
for v = 1:5
    res = ablation_results(2, v);
    plot(res.t, res.delta_d, 'Color', colors{v}, 'LineStyle', lines{v}, ...
        'LineWidth', widths{v}, 'DisplayName', var_labels{v});
end
title('Core-2: Spacing Error \delta_d Transient', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time [s]', 'Color', 'w'); ylabel('\delta_d [m]', 'Color', 'w');
ylim([0, 11]); xlim([0, 5]);
legend('Location', 'northeast', 'TextColor', 'w', 'Color', [0.1 0.1 0.1], 'FontSize', 8);

% Panel 4: Core-2 Applied Slew Rate Compliance
subplot(2, 2, 4);
set(gca, 'Color', 'k', 'XColor', 'w', 'YColor', 'w', 'GridColor', [0.3 0.3 0.3]);
hold on; grid on;
yline(0.25, 'w--', 'LineWidth', 1.5, 'DisplayName', 'Slew Limit \Delta u_{max}=0.25');
for v = 1:5
    res = ablation_results(2, v);
    plot(res.t, abs(res.delta_u_applied), 'Color', colors{v}, 'LineStyle', lines{v}, ...
        'LineWidth', widths{v}, 'DisplayName', var_labels{v});
end
title('Core-2: Applied Slew Rate Compliance |\Delta u_{app}|', 'Color', 'w', 'FontSize', 12, 'FontWeight', 'bold');
xlabel('Time [s]', 'Color', 'w'); ylabel('|\Delta u_{app}| [m/s^2]', 'Color', 'w');
ylim([0, 0.45]); xlim([0, 5]);
legend('Location', 'northeast', 'TextColor', 'w', 'Color', [0.1 0.1 0.1], 'FontSize', 8);

out_png = 'C:\Users\avina\.gemini\antigravity\brain\7425308c-5b2a-47c8-9cac-568ed3254a15\stage46_ablation_plots.png';
exportgraphics(fig, out_png, 'Resolution', 300);
fprintf('>> Saved Stage 46 Ablation plot: %s\n', out_png);

close(fig);
