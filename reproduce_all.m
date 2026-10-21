% REPRODUCE_ALL.M
% Master Reproduction Driver for:
% "Trajectory-Dependent Uncertainty Propagation and Applied-Input-Constrained
%  Tube MPC for Adaptive Cruise Control" (IEEE Transactions Submission)
%
% This script allows researchers to reproduce:
%   1. Table I & Table II: 5-Variant Ablation Study across 4 standardized benchmarks
%   2. Table III: 16-Case Emergency Braking Matrix across 3 controllers
%   3. Figure 3 & Figure 4: Ablation and Tripartite comparative trajectory plots
%   4. Sensitivity Audit: Acceleration drift parameter invariance (da = 0.20 vs 1.0921)
%   5. LMI Verification: Exterior multiplier and interior near-boundary certificate audit
%
% Usage:
%   Simply run `reproduce_all` in MATLAB. An interactive prompt will guide you,
%   or set `mode_override` to 1, 2, 3, 4, or 5 for headless execution.

if exist('mode_override', 'var')
    saved_mode = mode_override;
else
    saved_mode = [];
end
clearvars -except saved_mode; clc;
fprintf('========================================================================\n');
fprintf('  REPRODUCIBILITY MASTER DRIVER: TUBE MPC FOR ACC\n');
fprintf('  Manuscript: "Trajectory-Dependent Uncertainty Propagation and\n');
fprintf('               Applied-Input-Constrained Tube MPC for ACC"\n');
fprintf('========================================================================\n\n');

% Set paths relative to package root
pkg_root = fileparts(mfilename('fullpath'));
cd(pkg_root);
addpath(pkg_root);
addpath(fullfile(pkg_root, 'controllers'));
addpath(fullfile(pkg_root, 'data'));
addpath(fullfile(pkg_root, 'models'));
addpath(fullfile(pkg_root, 'scripts'));
addpath(fullfile(pkg_root, 'manuscript'));

parent_dir = fileparts(pkg_root);
if exist(fullfile(pkg_root, 'tools', 'YALMIP'), 'dir')
    addpath(genpath(fullfile(pkg_root, 'tools', 'YALMIP')));
    addpath(genpath(fullfile(pkg_root, 'tools', 'sdpt3')));
elseif exist(fullfile(parent_dir, 'tools', 'YALMIP'), 'dir')
    addpath(genpath(fullfile(parent_dir, 'tools', 'YALMIP')));
    addpath(genpath(fullfile(parent_dir, 'tools', 'sdpt3')));
end

fprintf('Package environment initialized successfully.\n\n');
fprintf('Available reproduction options:\n');
fprintf('  [1] Re-generate Table I & Table II Figures & Metrics from Raw Data (Fast, <2 sec)\n');
fprintf('  [2] Re-generate Table III 16-Case Emergency Braking Summary (Fast, <1 sec)\n');
fprintf('  [3] Re-verify Acceleration-Drift Sensitivity (da=0.20 vs 1.0921) (~30 sec)\n');
fprintf('  [4] Re-verify LMI Certificates & Multiplier Margins (<5 sec)\n');
fprintf('  [5] Run Full 5-Variant Simulation Ablation Study (20 simulations, ~4-6 min)\n');
fprintf('  [6] Run Full 16-Case Emergency Braking Matrix (48 simulations, ~8-12 min)\n\n');

if isempty(saved_mode)
    prompt = 'Select an option [1-6] (Default = 1): ';
    try
        user_choice = input(prompt);
    catch
        user_choice = 1;
    end
    if isempty(user_choice)
        user_choice = 1;
    end
else
    user_choice = saved_mode;
end

switch user_choice
    case 1
        fprintf('\n--- Option 1: Displaying Table I & II Metrics and Plotting Figures ---\n');
        load('stage46_ablation_results.mat', 'ablation_results');
        
        fprintf('\nTABLE I: CORE-3 MODERATE BRAKING BENCHMARK (a_p = -1.0 m/s^2):\n');
        fprintf('%-25s | %-10s | %-10s | %-10s | %-10s | %-10s\n', ...
            'Metric', 'Var-A', 'Var-B', 'Var-C', 'Var-D', 'Var-E');
        fprintf('%s\n', repmat('-', 1, 85));
        
        c3 = ablation_results(3, :);
        fprintf('%-25s | %9.1f%% | %9.1f%% | %9.1f%% | %9.1f%% | %9.1f%%\n', ...
            'Feasibility Ratio', c3(1).feas_pct, c3(2).feas_pct, c3(3).feas_pct, c3(4).feas_pct, c3(5).feas_pct);
        fprintf('%-25s | %10d | %10d | %10d | %10d | %10d\n', ...
            'Fallback Steps', c3(1).n_fallback, c3(2).n_fallback, c3(3).n_fallback, c3(4).n_fallback, c3(5).n_fallback);
        fprintf('%-25s | %10.4f | %10.4f | %10.4f | %10.4f | %10.4f\n', ...
            'Max Slew Viol (m/s^2)', c3(1).max_slew_viol, c3(2).max_slew_viol, c3(3).max_slew_viol, c3(4).max_slew_viol, c3(5).max_slew_viol);
        fprintf('%-25s | %+10.4f | %+10.4f | %+10.4f | %+10.4f | %+10.4f\n', ...
            'Min Interval w_min', c3(1).min_w, c3(2).min_w, c3(3).min_w, c3(4).min_w, c3(5).min_w);
        fprintf('%-25s | %10.4f | %10.4f | %10.4f | %10.4f | %10.4f\n', ...
            'Peak Sigma', c3(1).peak_sig, c3(2).peak_sig, c3(3).peak_sig, c3(4).peak_sig, c3(5).peak_sig);
        fprintf('%-25s | %10.4f | %10.4f | %10.4f | %10.4f | %10.4f\n', ...
            'Spacing RMSE (m)', c3(1).spacing_rmse, c3(2).spacing_rmse, c3(3).spacing_rmse, c3(4).spacing_rmse, c3(5).spacing_rmse);
        fprintf('%-25s | %10.4f | %10.4f | %10.4f | %10.4f | %10.4f\n', ...
            'Total Variation TV(u)', c3(1).total_variation, c3(2).total_variation, c3(3).total_variation, c3(4).total_variation, c3(5).total_variation);
            
        fprintf('\nTABLE II: CORE-2 SPACING GAP TRANSIENT BENCHMARK (x0 = [-10.0, 0, 0]^T):\n');
        fprintf('%-25s | %-10s | %-10s | %-10s | %-10s | %-10s\n', ...
            'Metric', 'Var-A', 'Var-B', 'Var-C', 'Var-D', 'Var-E');
        fprintf('%s\n', repmat('-', 1, 85));
        
        c2 = ablation_results(2, :);
        fprintf('%-25s | %9.1f%% | %9.1f%% | %9.1f%% | %9.1f%% | %9.1f%%\n', ...
            'Feasibility Ratio', c2(1).feas_pct, c2(2).feas_pct, c2(3).feas_pct, c2(4).feas_pct, c2(5).feas_pct);
        fprintf('%-25s | %10d | %10d | %10d | %10d | %10d\n', ...
            'Fallback Steps', c2(1).n_fallback, c2(2).n_fallback, c2(3).n_fallback, c2(4).n_fallback, c2(5).n_fallback);
        fprintf('%-25s | %10.4f | %10.4f | %10.4f | %10.4f | %10.4f\n', ...
            'Max Slew Viol (m/s^2)', c2(1).max_slew_viol, c2(2).max_slew_viol, c2(3).max_slew_viol, c2(4).max_slew_viol, c2(5).max_slew_viol);
        fprintf('%-25s | %+10.4f | %+10.4f | %+10.4f | %+10.4f | %+10.4f\n', ...
            'Min Interval w_min', c2(1).min_w, c2(2).min_w, c2(3).min_w, c2(4).min_w, c2(5).min_w);
        fprintf('%-25s | %10.4f | %10.4f | %10.4f | %10.4f | %10.4f\n', ...
            'Spacing RMSE (m)', c2(1).spacing_rmse, c2(2).spacing_rmse, c2(3).spacing_rmse, c2(4).spacing_rmse, c2(5).spacing_rmse);
        fprintf('%-25s | %10.4f | %10.4f | %10.4f | %10.4f | %10.4f\n', ...
            'Total Variation TV(u)', c2(1).total_variation, c2(2).total_variation, c2(3).total_variation, c2(4).total_variation, c2(5).total_variation);
            
        fprintf('\nGenerating Figure 3 (Ablation trajectories)...\n');
        run('stage46_plot_ablation.m');
        
    case 2
        fprintf('\n--- Option 2: Table III 16-Case Emergency Braking Summary ---\n');
        load('stage45_extended_results.mat', 'ext_results');
        fprintf('%-6s | %-36s | %-20s | %-20s | %-20s\n', ...
            'Case', 'Scenario Description', 'Ctrl-A (MaxSlew,lmin)', 'Ctrl-B (MaxSlew,lmin)', 'Ctrl-C (MaxSlew,lmin)');
        fprintf('%s\n', repmat('-', 1, 115));
        for c = 1:16
            rA = ext_results(c);
            rB = ext_results(16 + c);
            rC = ext_results(32 + c);
            fprintf('Case%-2d | %-36s | %7.4f, %5.2fm   | %7.4f, %5.2fm   | %7.4f, %5.2fm\n', ...
                c, rA.case_name, ...
                rA.max_slew_viol, rA.min_clearance, ...
                rB.max_slew_viol, rB.min_clearance, ...
                rC.max_slew_viol, rC.min_clearance);
        end
        
    case 3
        fprintf('\n--- Option 3: Re-running Acceleration-Drift Sensitivity Test ---\n');
        run('stage46_test_da_comparison.m');
        
    case 4
        fprintf('\n--- Option 4: Re-auditing LMI Certificates ---\n');
        run('stage44e_lmi_audit_and_reproducibility.m');
        
    case 5
        fprintf('\n--- Option 5: Running Full 5-Variant Simulation Ablation Study ---\n');
        run('stage46_run_ablation_study.m');
        
    case 6
        fprintf('\n--- Option 6: Running Full 16-Case Emergency Braking Matrix ---\n');
        run('stage45_extended_comparison.m');
        
    otherwise
        fprintf('Invalid option selected.\n');
end

fprintf('\nDone.\n');
