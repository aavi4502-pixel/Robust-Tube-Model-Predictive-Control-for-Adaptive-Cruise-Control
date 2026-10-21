% INIT_WORKSPACE.M
% Initializes all required base workspace variables for ACC Simulink models.
% Run this before opening or simulating ACC_Stage43GE_Experimental.slx.

ws_root = fileparts(mfilename('fullpath'));
if isempty(ws_root), ws_root = pwd; end
cd(ws_root);
addpath(ws_root);
addpath(fullfile(ws_root, 'controllers'));
addpath(fullfile(ws_root, 'data'));
addpath(fullfile(ws_root, 'models'));
addpath(fullfile(ws_root, 'scripts'));

parent_dir = fileparts(ws_root);
if exist(fullfile(ws_root, 'tools', 'YALMIP'), 'dir')
    addpath(genpath(fullfile(ws_root, 'tools', 'YALMIP')));
    addpath(genpath(fullfile(ws_root, 'tools', 'sdpt3')));
elseif exist(fullfile(parent_dir, 'tools', 'YALMIP'), 'dir')
    addpath(genpath(fullfile(parent_dir, 'tools', 'YALMIP')));
    addpath(genpath(fullfile(parent_dir, 'tools', 'sdpt3')));
end

fprintf('Initializing base workspace for ACC Simulink simulation...\n');

% 1. Load plant, baseline, and scenario parameters
c_base = load('baseline_provisional_r10_params.mat');
fnames = fieldnames(c_base);
for k = 1:length(fnames)
    assignin('base', fnames{k}, c_base.(fnames{k}));
end

% 2. Load optimal co-designed tube gain (Candidate 2)
c_lmi = load('stage44c_lmi_candidate.mat', 'lmi_results');
cand2 = c_lmi.lmi_results(2);

assignin('base', 'K_tube_active', cand2.K);
assignin('base', 'tau_h', c_base.tau_h);
assignin('base', 'l0', c_base.l0);
assignin('base', 'Ts', c_base.Ts);

% 3. Set default simulation scenario: Moderate Braking (Benchmark 3)
T_sim = 5.0;
Ts = c_base.Ts;
t_grid = (0:Ts:T_sim)';
n_steps = length(t_grid);

x0 = [0; 0; 0]; % Default equilibrium initial state
ap_profile = zeros(n_steps, 1);
ap_profile(t_grid >= 1.0 & t_grid <= 3.0) = -1.0; % -1.0 m/s^2 braking at [1, 3]s
sim_ap_profile = [t_grid, ap_profile];

assignin('base', 'x0', x0);
assignin('base', 'sim_ap_profile', sim_ap_profile);
assignin('base', 't_gust_on', 999);  % No wind by default
assignin('base', 't_gust_off', 999);
assignin('base', 'stage45_active_controller', 'ABL_E'); % Variant E (Proposed Controller)

fprintf('Base workspace successfully initialized!\n');
fprintf('  - Initial state x0:             [0; 0; 0]\n');
fprintf('  - Sampling time Ts:             %.2f s\n', Ts);
fprintf('  - Lead acceleration profile:    sim_ap_profile loaded (-1.0 m/s^2 at [1, 3]s)\n');
fprintf('  - Wind gust window:             t_gust_on = 999 (disabled)\n');
fprintf('  - Active tube gain:             K_tube_active = [%.4f, %.4f, %.4f]\n', cand2.K);
fprintf('  - Active controller mode:       ABL_E (Proposed Tube MPC with Step-0 Conditioning)\n\n');
fprintf('You can now safely open and run ACC_Stage43GE_Experimental in Simulink.\n');
