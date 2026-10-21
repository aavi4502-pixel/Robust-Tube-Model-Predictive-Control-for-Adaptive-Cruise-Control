% STAGE46_RUN_ABLATION_STUDY.M
% Stage 46B: Systematic 5-Variant Ablation Study across Core Benchmarks
% Isolates the independent contributions of:
%   1. Trajectory-Dependent Bound Propagation (Static vs Trajectory)
%   2. Instantaneous Ancillary-Aware Step-0 Constraints (No vs Yes)
%   3. LMI Gain Co-Design (Fixed K_paper vs Co-Designed K_opt)
%
% All variants are evaluated under identical standardized slew-invariant fallback!

clear; clc;
fprintf('========================================================================\n');
fprintf('  STAGE 46B: SYSTEMATIC 5-VARIANT ABLATION STUDY\n');
fprintf('========================================================================\n\n');

this_file = mfilename('fullpath');
if ~isempty(this_file)
    s_dir = fileparts(this_file);
    pkg_dir = fileparts(s_dir);
else
    pkg_dir = pwd;
end
cd(pkg_dir);
addpath(pkg_dir);
addpath(fullfile(pkg_dir, 'controllers'));
addpath(fullfile(pkg_dir, 'data'));
addpath(fullfile(pkg_dir, 'models'));
addpath(fullfile(pkg_dir, 'scripts'));

parent_dir = fileparts(pkg_dir);
if exist(fullfile(pkg_dir, 'tools', 'YALMIP'), 'dir')
    addpath(genpath(fullfile(pkg_dir, 'tools', 'YALMIP')));
    addpath(genpath(fullfile(pkg_dir, 'tools', 'sdpt3')));
elseif exist(fullfile(parent_dir, 'tools', 'YALMIP'), 'dir')
    addpath(genpath(fullfile(parent_dir, 'tools', 'YALMIP')));
    addpath(genpath(fullfile(parent_dir, 'tools', 'sdpt3')));
end

% 1. Load frozen baseline and candidate parameters
c_base = load('baseline_provisional_r10_params.mat');
c_lmi  = load('stage44c_lmi_candidate.mat', 'lmi_results');
cand2  = c_lmi.lmi_results(2); % K_opt = [1.2345, -0.5985, -1.2588]

K_paper = c_base.K_tube;
K_opt   = cand2.K;

% Ensure base workspace has required plant parameters
fnames = fieldnames(c_base);
for k = 1:length(fnames)
    assignin('base', fnames{k}, c_base.(fnames{k}));
end

tau_h = c_base.tau_h;
l0    = c_base.l0;

% 2. Define Ablation Variants
variants = struct();

% Variant A: Static Bound, No Ancillary Step-0, Fixed Gain
variants(1).id         = 'ABL_A';
variants(1).short_name = 'Var-A (Static, NoStep0, K_paper)';
variants(1).bound_mode = 'Static (3.0 m/s^2)';
variants(1).step0_mode = 'No';
variants(1).gain_mode  = 'Fixed (K_paper)';
variants(1).K          = K_paper;

% Variant B: Trajectory Bound, No Ancillary Step-0, Fixed Gain
variants(2).id         = 'ABL_B';
variants(2).short_name = 'Var-B (Traj, NoStep0, K_paper)';
variants(2).bound_mode = 'Trajectory';
variants(2).step0_mode = 'No';
variants(2).gain_mode  = 'Fixed (K_paper)';
variants(2).K          = K_paper;

% Variant C: Static Bound, Ancillary Step-0, Fixed Gain
variants(3).id         = 'ABL_C';
variants(3).short_name = 'Var-C (Static, Step0, K_paper)';
variants(3).bound_mode = 'Static (3.0 m/s^2)';
variants(3).step0_mode = 'Yes';
variants(3).gain_mode  = 'Fixed (K_paper)';
variants(3).K          = K_paper;

% Variant D: Trajectory Bound, Ancillary Step-0, Fixed Gain
variants(4).id         = 'ABL_D';
variants(4).short_name = 'Var-D (Traj, Step0, K_paper)';
variants(4).bound_mode = 'Trajectory';
variants(4).step0_mode = 'Yes';
variants(4).gain_mode  = 'Fixed (K_paper)';
variants(4).K          = K_paper;

% Variant E: Trajectory Bound, Ancillary Step-0, Co-Designed Gain
variants(5).id         = 'ABL_E';
variants(5).short_name = 'Var-E (Traj, Step0, K_opt)';
variants(5).bound_mode = 'Trajectory';
variants(5).step0_mode = 'Yes';
variants(5).gain_mode  = 'Co-Designed (K_opt)';
variants(5).K          = K_opt;

n_var = length(variants);

% 3. Define Benchmark Scenarios (Identical to Stage 45 Core)
T_sim   = 5.0;
Ts      = c_base.Ts; % 0.1s
t_grid  = (0:Ts:T_sim)';
n_steps = length(t_grid);

benchmarks = struct();

% Scenario 1: Nominal Equilibrium
benchmarks(1).id         = 'Core-1';
benchmarks(1).name       = 'Core-1: Nominal Equilibrium';
benchmarks(1).x0         = [0; 0; 0];
benchmarks(1).ap         = zeros(n_steps, 1);
benchmarks(1).t_gust_on  = 999;
benchmarks(1).t_gust_off = 999;
benchmarks(1).desc       = 'Nominal equilibrium holding (x0 = 0, a_p = 0, w = 0)';

% Scenario 2: Spacing Gap Transient
benchmarks(2).id         = 'Core-2';
benchmarks(2).name       = 'Core-2: Spacing Gap Transient';
benchmarks(2).x0         = [5; 0; 0];
benchmarks(2).ap         = zeros(n_steps, 1);
benchmarks(2).t_gust_on  = 999;
benchmarks(2).t_gust_off = 999;
benchmarks(2).desc       = 'Initial spacing gap closing (x0 = [5, 0, 0], a_p = 0, w = 0)';

% Scenario 3: Moderate Lead Braking
benchmarks(3).id         = 'Core-3';
benchmarks(3).name       = 'Core-3: Moderate Lead Braking';
benchmarks(3).x0         = [0; 0; 0];
ap_mod = zeros(n_steps, 1);
ap_mod(t_grid >= 1.0 & t_grid <= 3.0) = -1.0;
benchmarks(3).ap         = ap_mod;
benchmarks(3).t_gust_on  = 999;
benchmarks(3).t_gust_off = 999;
benchmarks(3).desc       = 'Moderate lead deceleration (-1.0 m/s^2 at [1, 3]s)';

% Scenario 4: Severe Braking + Active Wind
benchmarks(4).id         = 'Core-4';
benchmarks(4).name       = 'Core-4: Severe Braking + Active Wind';
benchmarks(4).x0         = [2; 1; 0.5];
ap_sev = zeros(n_steps, 1);
ap_sev(t_grid >= 1.0 & t_grid <= 3.0) = -2.0;
benchmarks(4).ap         = ap_sev;
benchmarks(4).t_gust_on  = 0.0;
benchmarks(4).t_gust_off = 5.0;
benchmarks(4).desc       = 'Severe lead deceleration (-2.0 m/s^2) with continuous wind gust';

n_bm = length(benchmarks);

% 4. Load Model
mdl = 'ACC_Stage43GE_Experimental';
if ~bdIsLoaded(mdl)
    load_system(mdl);
end

ablation_results = struct();

fprintf('Executing Ablation Study (%d variants x %d benchmarks = %d simulations)...\n\n', ...
    n_var, n_bm, n_var * n_bm);

for b = 1:n_bm
    bm = benchmarks(b);
    fprintf('========================================================================\n');
    fprintf('  BENCHMARK %s: %s\n', bm.id, bm.name);
    fprintf('  Condition: %s\n', bm.desc);
    fprintf('========================================================================\n');
    
    assignin('base', 'x0', bm.x0);
    sim_ap_profile = [t_grid, bm.ap];
    assignin('base', 'sim_ap_profile', sim_ap_profile);
    assignin('base', 't_gust_on', bm.t_gust_on);
    assignin('base', 't_gust_off', bm.t_gust_off);
    
    for v = 1:n_var
        var_cfg = variants(v);
        fprintf('  -> Simulating %s...\n', var_cfg.short_name);
        
        assignin('base', 'stage45_active_controller', var_cfg.id);
        assignin('base', 'K_tube_active', var_cfg.K);
        
        % Clear persistent states
        clear('stage45_mpc_dispatcher');
        clear('stage46_ablation_controller');
        
        t_sim_start = tic;
        sim_out = sim(mdl, 'StopTime', num2str(T_sim));
        wall_sim_time = toc(t_sim_start);
        
        % Extract signals exactly matching Stage 45
        t_ctrl   = sim_out.dbg_umpc.Time(:);
        u_xi     = sim_out.dbg_umpc.Data(:);
        K_exi    = sim_out.dbg_kexi.Data(:);
        d_hat    = sim_out.dbg_dhat.Data(:);
        c_t      = K_exi - d_hat;
        
        u_raw    = squeeze(sim_out.out_ucmd_log(:));
        u_app    = squeeze(sim_out.out_usat_log(:));
        
        x_ctrl   = interp1(sim_out.tout, sim_out.out_x_log, t_ctrl);
        delta_d  = x_ctrl(:, 1);
        delta_v  = x_ctrl(:, 2);
        a_ego    = x_ctrl(:, 3);
        
        v_ego    = 20.0 + cumtrapz(t_ctrl, a_ego);
        clearance = delta_d + tau_h * v_ego + l0;
        min_clearance = min(clearance);
        
        qp_status   = sim_out.Log_qp_status(:);
        fb_active   = sim_out.Log_fallback_active(:);
        min_w_arr   = sim_out.Log_min_w(:);
        sigma_arr   = sim_out.Log_peak_sig(:);
        time_ms_arr = sim_out.Log_solve_time_ms(:);
        
        n_samples    = length(t_ctrl);
        n_infeasible = sum(qp_status > 0);
        n_fallback   = sum(fb_active > 0);
        feas_pct     = 100 * (1 - n_infeasible / n_samples);
        fallback_pct = 100 * (n_fallback / n_samples);
        
        % Slew metrics
        du_app_step = [0; diff(u_app)];
        max_slew_viol_app = max([0; max(abs(du_app_step) - 0.25)]);
        
        % Actuator bounds
        max_act_viol_raw = max([0; max(u_raw - 2.50); max(-3.00 - u_raw)]);
        max_act_viol_app = max([0; max(u_app - 2.50); max(-3.00 - u_app)]);
        
        % Tracking metrics
        spacing_rmse     = sqrt(mean(delta_d.^2));
        peak_spacing_err = max(abs(delta_d));
        control_energy   = sum(u_app.^2) * Ts;
        total_variation  = sum(abs(diff(u_app)));
        
        min_w       = min(min_w_arr);
        peak_sig    = max(sigma_arr);
        median_time = median(time_ms_arr(time_ms_arr > 0));
        if isempty(median_time) || isnan(median_time); median_time = 0; end
        
        fprintf('     Done in %.2fs: Feas=%5.1f%% | FB=%2d | SlewViol=%.2e | ActViol=%.2e | l_min=%.2fm | RMSE=%.3fm | Energy=%.2f | TV=%.3f | PeakSig=%.2f\n', ...
            wall_sim_time, feas_pct, n_fallback, max_slew_viol_app, max_act_viol_app, min_clearance, spacing_rmse, control_energy, total_variation, peak_sig);
        
        % Store results
        ablation_results(b, v).bm_id            = bm.id;
        ablation_results(b, v).bm_name          = bm.name;
        ablation_results(b, v).var_id           = var_cfg.id;
        ablation_results(b, v).var_short        = var_cfg.short_name;
        ablation_results(b, v).bound_mode       = var_cfg.bound_mode;
        ablation_results(b, v).step0_mode       = var_cfg.step0_mode;
        ablation_results(b, v).gain_mode        = var_cfg.gain_mode;
        ablation_results(b, v).feas_pct         = feas_pct;
        ablation_results(b, v).n_fallback       = n_fallback;
        ablation_results(b, v).fallback_pct     = fallback_pct;
        ablation_results(b, v).min_clearance    = min_clearance;
        ablation_results(b, v).spacing_rmse     = spacing_rmse;
        ablation_results(b, v).peak_spacing_err = peak_spacing_err;
        ablation_results(b, v).control_energy   = control_energy;
        ablation_results(b, v).total_variation  = total_variation;
        ablation_results(b, v).max_slew_viol    = max_slew_viol_app;
        ablation_results(b, v).max_act_viol     = max_act_viol_app;
        ablation_results(b, v).min_w            = min_w;
        ablation_results(b, v).peak_sig         = peak_sig;
        ablation_results(b, v).median_time_ms   = median_time;
        
        % Save trajectory data
        ablation_results(b, v).t                = t_ctrl;
        ablation_results(b, v).delta_d          = delta_d;
        ablation_results(b, v).u_applied        = u_app;
        ablation_results(b, v).delta_u_applied  = du_app_step;
    end
    fprintf('\n');
end

%% -----------------------------------------------------------------------
% DISPLAY CONSOLIDATED ABLATION TABLES
%% -----------------------------------------------------------------------
fprintf('====================================================================================================================================\n');
fprintf('  TABLE 1: STAGE 46B ABLATION MATRIX — PERFORMANCE & TRACKING\n');
fprintf('====================================================================================================================================\n\n');
fprintf(' %-8s | %-32s | Feas %% | FB Steps | Spacing RMSE | Peak Spacing | Control Energy | Total Var | min Headway\n', ...
    'Scenario', 'Variant');
fprintf('------------------------------------------------------------------------------------------------------------------------------------\n');
for b = 1:n_bm
    for v = 1:n_var
        ar = ablation_results(b, v);
        fprintf(' %-8s | %-32s | %5.1f%% |    %2d    |   %7.3f m   |   %7.3f m   |    %8.2f    |  %7.3f  |   %6.2f m\n', ...
            ar.bm_id, ar.var_short, ar.feas_pct, ar.n_fallback, ar.spacing_rmse, ar.peak_spacing_err, ...
            ar.control_energy, ar.total_variation, ar.min_clearance);
    end
    fprintf('------------------------------------------------------------------------------------------------------------------------------------\n');
end
fprintf('\n');

fprintf('====================================================================================================================================\n');
fprintf('  TABLE 2: STAGE 46B ABLATION MATRIX — CONSTRAINT COMPLIANCE & BOUND CONSERVATISM\n');
fprintf('====================================================================================================================================\n\n');
fprintf(' %-8s | %-32s | SlewViol(app) | ActViol(app) | min Width w_min | Peak Sigma | Median Time\n', ...
    'Scenario', 'Variant');
fprintf('------------------------------------------------------------------------------------------------------------------------------------\n');
for b = 1:n_bm
    for v = 1:n_var
        ar = ablation_results(b, v);
        fprintf(' %-8s | %-32s |   %8.2e    |   %8.2e   |     %7.3f     |   %7.2f  |   %5.1f ms\n', ...
            ar.bm_id, ar.var_short, ar.max_slew_viol, ar.max_act_viol, ar.min_w, ar.peak_sig, ar.median_time_ms);
    end
    fprintf('------------------------------------------------------------------------------------------------------------------------------------\n');
end
fprintf('\n');

save('stage46_ablation_results.mat', 'ablation_results', 'benchmarks', 'variants');
fprintf('>> Saved Stage 46 Ablation archive: stage46_ablation_results.mat\n');

close_system(mdl, 0);
