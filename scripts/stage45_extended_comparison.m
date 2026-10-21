%% stage45_extended_comparison.m
% Stage 45: Part 2 - Extended 16-Case Emergency Braking Tri-Partite Comparison
%
% Compares Controller A, Controller B, and Controller C across all 16 Stage 44E cases.
%
% Strictly evaluates:
%   - Feasibility %, Fallback steps
%   - Actuator and Slew compliance
%   - Min clearance l_min and min TTC
%   - Control effort and tracking metrics
%
% Saves complete dataset to stage45_extended_results.mat.

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

fprintf('========================================================================\n');
fprintf('  STAGE 45: EXTENDED 16-CASE TRI-PARTITE EMERGENCY BRAKING COMPARISON\n');
fprintf('========================================================================\n\n');

load('baseline_provisional_r10_params.mat');
load('stage44e_braking_study_results.mat', 'cases');
mdl = 'ACC_Stage43GE_Experimental';
load_system(mdl);

% Configure model
set_param([mdl, '/ACC_Controller/Eq64_MPC'], 'MATLABFcn', 'stage45_mpc_dispatcher(u)');
set_param([mdl, '/ACC_Controller/Eq27_TubeGain'], 'Gain', 'K_tube_active');

T_sim = 5.0;
t_grid = (0:Ts:T_sim)';
n_steps = length(t_grid);
n_cases = length(cases);

ctrl_configs = struct();
ctrl_configs(1).id   = 'A';
ctrl_configs(1).name = 'Controller A (Nominal)';
ctrl_configs(1).K    = [0, 0, 0];

ctrl_configs(2).id   = 'B';
ctrl_configs(2).name = 'Controller B (Paper Static)';
ctrl_configs(2).K    = [2.4855, -0.2200, -2.0959];

ctrl_configs(3).id   = 'C';
ctrl_configs(3).name = 'Controller C (Proposed)';
ctrl_configs(3).K    = [1.2345, -0.5985, -1.2588];

n_ctrls = length(ctrl_configs);
ext_results = struct();

fprintf('Running 16 cases across 3 controllers (48 simulations total)...\n\n');

for c = 1:n_ctrls
    cfg = ctrl_configs(c);
    fprintf('------------------------------------------------------------------------\n');
    fprintf('  BENCHMARKING %s\n', cfg.name);
    fprintf('------------------------------------------------------------------------\n');
    
    assignin('base', 'stage45_active_controller', cfg.id);
    assignin('base', 'K_tube_active', cfg.K);
    
    for k = 1:n_cases
        cs = cases(k);
        assignin('base', 'x0', cs.x0);
        
        ap_vec = zeros(n_steps, 1);
        ap_vec(t_grid >= 1.0 & t_grid <= 3.0) = cs.ap_mag;
        sim_ap_profile = [t_grid, ap_vec];
        assignin('base', 'sim_ap_profile', sim_ap_profile);
        
        if cs.wind
            assignin('base', 't_gust_on', 0.0);
            assignin('base', 't_gust_off', 5.0);
        else
            assignin('base', 't_gust_on', 999);
            assignin('base', 't_gust_off', 999);
        end
        
        % Clear solver states
        clear('stage45_mpc_dispatcher');
        clear('stage45_controller_a_nominal');
        clear('stage45_controller_b_paper');
        clear('stage45_controller_c_proposed');
        clear('stage44c_mpc_solver');
        
        try
            sim_out = sim(mdl, 'StopTime', num2str(T_sim));
            sim_ok = true;
        catch ME
            fprintf('  ERROR Case %d: %s\n', k, ME.message);
            sim_ok = false;
        end
        
        if sim_ok
            t_ctrl = sim_out.dbg_umpc.Time(:);
            u_app  = squeeze(sim_out.out_usat_log(:));
            u_raw  = squeeze(sim_out.out_ucmd_log(:));
            qp_st  = sim_out.Log_qp_status(:);
            fb_act = sim_out.Log_fallback_active(:);
            
            x_ctrl = interp1(sim_out.tout, sim_out.out_x_log, t_ctrl);
            delta_d = x_ctrl(:, 1);
            delta_v = x_ctrl(:, 2);
            a_ego   = x_ctrl(:, 3);
            
            v_ego = 20.0 + cumtrapz(t_ctrl, a_ego);
            clearance = delta_d + tau_h * v_ego + l0;
            min_clearance = min(clearance);
            
            % Slew rate
            du_app = [0; diff(u_app)];
            max_slew_viol = max([0; max(abs(du_app) - 0.25)]);
            max_act_viol  = max([0; max(u_app - 2.50); max(-3.00 - u_app)]);
            
            n_fb = sum(fb_act > 0);
            fb_pct = 100 * n_fb / length(t_ctrl);
            feas_pct = 100 * (1 - sum(qp_st > 0) / length(t_ctrl));
            
            % TTC
            appr = (delta_v < -0.1 & clearance > 0);
            if any(appr)
                min_ttc = min(clearance(appr) ./ (-delta_v(appr)));
            else
                min_ttc = Inf;
            end
            
            % Tracking and control metrics
            rmse_d = sqrt(mean(delta_d.^2));
            energy = sum(u_app.^2) * Ts;
            totvar = sum(abs(diff(u_app)));
            
            ext_results(k, c).case_id       = k;
            ext_results(k, c).case_name     = cs.name;
            ext_results(k, c).ctrl_id       = cfg.id;
            ext_results(k, c).ctrl_name     = cfg.name;
            ext_results(k, c).feas_pct      = feas_pct;
            ext_results(k, c).fb_pct        = fb_pct;
            ext_results(k, c).n_fb          = n_fb;
            ext_results(k, c).min_clearance = min_clearance;
            ext_results(k, c).min_ttc       = min_ttc;
            ext_results(k, c).max_slew_viol = max_slew_viol;
            ext_results(k, c).max_act_viol  = max_act_viol;
            ext_results(k, c).rmse_d        = rmse_d;
            ext_results(k, c).energy        = energy;
            ext_results(k, c).totvar        = totvar;
            
            fprintf('  Case %2d: Feas=%5.1f%% | FB=%4.1f%% | SlewViol=%.2e | ActViol=%.2e | min(l)=%5.2fm\n', ...
                k, feas_pct, fb_pct, max_slew_viol, max_act_viol, min_clearance);
        end
    end
    fprintf('\n');
end

close_system(mdl, 0);

%% -----------------------------------------------------------------------
% AGGREGATED STATISTICAL SUMMARY
%% -----------------------------------------------------------------------
fprintf('========================================================================================================================\n');
fprintf('  AGGREGATED STATISTICAL SUMMARY ACROSS ALL 16 EMERGENCY BRAKING CASES\n');
fprintf('========================================================================================================================\n\n');

fprintf(' Controller                    | Mean Feas %% | Mean FB %% | Max Slew Viol | Max Act Viol | Mean min(l) | Min Clearance All\n');
fprintf('------------------------------------------------------------------------------------------------------------------------\n');
for c = 1:n_ctrls
    c_res = ext_results(:, c);
    mean_feas = mean([c_res.feas_pct]);
    mean_fb   = mean([c_res.fb_pct]);
    max_slew  = max([c_res.max_slew_viol]);
    max_act   = max([c_res.max_act_viol]);
    mean_minl = mean([c_res.min_clearance]);
    min_l_all = min([c_res.min_clearance]);
    
    fprintf(' %-29s |    %5.1f%%   |   %5.1f%%  |   %8.2e    |   %8.2e   |   %5.2f m   |     %5.2f m\n', ...
        ctrl_configs(c).name, mean_feas, mean_fb, max_slew, max_act, mean_minl, min_l_all);
end
fprintf('\n');

save('stage45_extended_results.mat', 'ext_results', 'cases', 'ctrl_configs');
fprintf('>> Saved Stage 45 extended comparison archive: stage45_extended_results.mat\n');
