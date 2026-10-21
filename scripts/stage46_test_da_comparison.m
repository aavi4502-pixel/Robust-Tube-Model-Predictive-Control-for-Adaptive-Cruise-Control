% STAGE46_TEST_DA_COMPARISON.M
% Evaluates Variant E under delta_a_bar = 0.20 vs delta_a_bar = 1.0921 (Physical Envelope)
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

c_base = load('baseline_provisional_r10_params.mat');
fnames = fieldnames(c_base);
for k = 1:length(fnames)
    assignin('base', fnames{k}, c_base.(fnames{k}));
end

c_lmi = load('stage44c_lmi_candidate.mat', 'lmi_results');
cand2 = c_lmi.lmi_results(2); % K_opt = [1.2345, -0.5985, -1.2588]

tau_h = c_base.tau_h;
l0    = c_base.l0;
assignin('base', 'tau_h', tau_h);
assignin('base', 'l0', l0);
assignin('base', 'K_tube_active', cand2.K);

mdl = 'ACC_Stage43GE_Experimental';
if ~bdIsLoaded(mdl)
    load_system(mdl);
end

T_sim   = 5.0;
Ts      = c_base.Ts;
t_grid  = (0:Ts:T_sim)';
n_steps = length(t_grid);

% Scenario: Benchmark 3 (Moderate Braking)
x0 = [0; 0; 0];
ap_mod = zeros(n_steps, 1);
ap_mod(t_grid >= 1.0 & t_grid <= 3.0) = -1.0;
sim_ap_profile = [t_grid, ap_mod];

assignin('base', 'x0', x0);
assignin('base', 'sim_ap_profile', sim_ap_profile);
assignin('base', 't_gust_on', 999);
assignin('base', 't_gust_off', 999);
assignin('base', 'stage45_active_controller', 'ABL_E');

mdl = 'ACC_Stage43GE_Experimental';
load_system(mdl);

da_test_values = [0.20, 1.0921];
results_cmp = struct();

for i = 1:length(da_test_values)
    da_val = da_test_values(i);
    fprintf('\n=======================================================\n');
    fprintf('  Testing Variant E with delta_a_bar = %.4f m/s^2\n', da_val);
    fprintf('=======================================================\n');
    
    % Clear persistent states
    clear('stage45_mpc_dispatcher');
    clear('stage46_ablation_controller');
    
    assignin('base', 'override_delta_a_bar', da_val);
    
    sim_out = sim(mdl, 'StopTime', num2str(T_sim));
    
    t_ctrl    = sim_out.dbg_umpc.Time(:);
    u_app     = squeeze(sim_out.out_usat_log(:));
    qp_status = sim_out.Log_qp_status(:);
    fb_active = sim_out.Log_fallback_active(:);
    min_w_arr = sim_out.Log_min_w(:);
    sigma_arr = sim_out.Log_peak_sig(:);
    
    n_samples    = length(t_ctrl);
    n_infeasible = sum(qp_status > 0);
    n_fallback   = sum(fb_active > 0);
    feas_pct     = 100 * (1 - n_infeasible / n_samples);
    
    du_app_step = [0; diff(u_app)];
    max_slew_viol_app = max([0; max(abs(du_app_step) - 0.25)]);
    
    min_w    = min(min_w_arr);
    peak_sig = max(sigma_arr);
    
    results_cmp(i).da = da_val;
    results_cmp(i).feas_pct = feas_pct;
    results_cmp(i).n_fallback = n_fallback;
    results_cmp(i).max_slew_viol_app = max_slew_viol_app;
    results_cmp(i).min_w = min_w;
    results_cmp(i).peak_sig = peak_sig;
    
    fprintf('  Feasibility:   %.1f%% (%d / %d feasible)\n', feas_pct, n_samples - n_infeasible, n_samples);
    fprintf('  Fallback:      %d steps\n', n_fallback);
    fprintf('  Max Slew Viol: %.6e m/s^2\n', max_slew_viol_app);
    fprintf('  Min Width w0:  %.4f m/s^2\n', min_w);
    fprintf('  Peak Sigma:    %.4f\n', peak_sig);
end

save('stage46_da_comparison_results.mat', 'results_cmp');
fprintf('\nComparison test complete and saved to stage46_da_comparison_results.mat\n');
