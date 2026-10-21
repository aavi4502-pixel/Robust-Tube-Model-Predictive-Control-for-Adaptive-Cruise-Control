function out = stage45_controller_a_nominal(in_vec)
% STAGE45_CONTROLLER_A_NOMINAL
% Controller A: Standard Nominal MPC without robust tube tightening or ancillary feedback.
%
% Inputs: in_vec(1:11)
% Outputs: out(1:6) = [u_xi_cmd; qp_status; fallback_active; min_w; peak_sig; solve_time_ms]

persistent precomp;

if isempty(precomp)
    this_file = mfilename('fullpath');
    if ~isempty(this_file)
        c_dir = fileparts(this_file);
        p_dir = fileparts(c_dir);
        if exist(fullfile(p_dir, 'data'), 'dir')
            addpath(fullfile(p_dir, 'data'));
        end
        if exist(fullfile(p_dir, 'tools', 'YALMIP'), 'dir')
            addpath(genpath(fullfile(p_dir, 'tools', 'YALMIP')));
            addpath(genpath(fullfile(p_dir, 'tools', 'sdpt3')));
        elseif exist(fullfile(fileparts(p_dir), 'tools', 'YALMIP'), 'dir')
            addpath(genpath(fullfile(fileparts(p_dir), 'tools', 'YALMIP')));
            addpath(genpath(fullfile(fileparts(p_dir), 'tools', 'sdpt3')));
        end
    end
    
    c_const = load('stage43e_candidate_constraints.mat', 'Hx_corr', 'hx_corr');
    c_qp    = load('stage43a_qp_matrices.mat', 'Phi', 'Gamma', 'Q_bar', 'R_bar', 'W_bar', ...
                   'delta_u_max', 'delta_u_min');
    c_base  = load('baseline_provisional_r10_params.mat', 'A_c', 'B_c', 'Ts');
    
    N = 6;
    precomp.N = N;
    precomp.Ts = c_base.Ts;
    precomp.u_max = 2.50;
    precomp.u_min = -3.00;
    precomp.delta_u_max = c_qp.delta_u_max; % 0.25
    precomp.delta_u_min = c_qp.delta_u_min; % -0.25
    
    % ZOH discretization of lead acceleration channel B_ap = [0; 1; 0]
    B_ap = [0; 1; 0];
    nx = 3; nu = 1; n_ap = 1;
    M_aug = [c_base.A_c, c_base.B_c, B_ap; zeros(nu + n_ap, nx + nu + n_ap)];
    Md_aug = expm(M_aug * c_base.Ts);
    Gamma_ap = Md_aug(1:nx, nx+nu+1:end);
    precomp.Gamma_ap = Gamma_ap;
    
    Phi = c_qp.Phi;
    Gamma = c_qp.Gamma;
    precomp.Phi = Phi;
    precomp.Gamma = Gamma;
    
    S_x_N = zeros(3*N, 3);
    for k = 1:N; S_x_N((k-1)*3 + (1:3), :) = Phi^k; end
    
    S_full_N = zeros(3*N, N);
    for k = 1:N
        for j = 1:k; S_full_N((k-1)*3 + (1:3), j) = (Phi^(k - j)) * Gamma; end
    end
    S_u_N = S_full_N * ones(N, 1);
    
    S_ap_N = zeros(3*N, N);
    for k = 1:N
        for j = 1:k; S_ap_N((k-1)*3 + (1:3), j) = (Phi^(k - j)) * Gamma_ap; end
    end
    
    C_u_bar_N = tril(ones(N, N));
    L_delta_N = C_u_bar_N;
    G_delta_N = S_full_N * C_u_bar_N;
    
    Q_step = c_qp.Q_bar(1:3, 1:3);
    R_step = c_qp.R_bar(1, 1);
    W_step = c_qp.W_bar(1, 1);
    
    Q_bar_N = kron(eye(N), Q_step);
    R_bar_N = kron(eye(N), R_step);
    W_bar_N = kron(eye(N), W_step);
    
    H_qp_N = 2 * (G_delta_N' * Q_bar_N * G_delta_N + L_delta_N' * R_bar_N * L_delta_N + W_bar_N);
    
    Hx_bar_N = kron(eye(N), c_const.Hx_corr);
    A_ineq_N = [ eye(N);
                -eye(N);
                 C_u_bar_N;
                -C_u_bar_N;
                 Hx_bar_N * G_delta_N ];
             
    precomp.S_x = S_x_N;
    precomp.S_u = S_u_N;
    precomp.S_ap = S_ap_N;
    precomp.G_delta = G_delta_N;
    precomp.L_delta = L_delta_N;
    precomp.C_u_bar = C_u_bar_N;
    precomp.H_qp = H_qp_N;
    precomp.Q_bar = Q_bar_N;
    precomp.R_bar = R_bar_N;
    precomp.hx_corr = c_const.hx_corr;
    precomp.Hx_bar = Hx_bar_N;
    precomp.A_ineq = A_ineq_N;
end

x_nom      = in_vec(1:3);
d_hat      = in_vec(7);
u_xi_prev  = in_vec(8);
a_p        = in_vec(9);
t          = in_vec(10);
if length(in_vec) >= 11
    u_app_prev = in_vec(11);
else
    u_app_prev = u_xi_prev;
end

t_start = tic;

N = precomp.N;
u_max = precomp.u_max;
u_min = precomp.u_min;
delta_u_max = precomp.delta_u_max;
delta_u_min = precomp.delta_u_min;

% Prediction vector and linear cost
Ap_vec = a_p * ones(N, 1);
f_xi = precomp.S_x * x_nom + precomp.S_u * u_xi_prev + precomp.S_ap * Ap_vec;
c_qp = 2 * (precomp.G_delta' * precomp.Q_bar * f_xi + precomp.L_delta' * precomp.R_bar * (ones(N, 1) * u_xi_prev));

% Nominal Slew & Input Bounds (no tightening!)
b_slew = [ delta_u_max * ones(N, 1); -delta_u_min * ones(N, 1) ];
% Bound first step against u_app_prev for physical consistency
b_slew(1)   = min(delta_u_max, u_max - u_app_prev);
b_slew(N+1) = min(-delta_u_min, -(u_min - u_app_prev));

b_input = [ u_max * ones(N, 1) - ones(N, 1)*u_xi_prev;
           -u_min * ones(N, 1) + ones(N, 1)*u_xi_prev ];
b_state = kron(ones(N, 1), precomp.hx_corr) - precomp.Hx_bar * f_xi;

b_nom = [ b_slew; b_input; b_state ];

yalmip('clear');
dU_var = sdpvar(N, 1);
ops_silent = sdpsettings('solver', 'sdpt3', 'verbose', 0);
sol_qp = optimize([ precomp.A_ineq * dU_var <= b_nom ], ...
                  0.5 * dU_var' * precomp.H_qp * dU_var + c_qp' * dU_var, ops_silent);

solve_time_ms = toc(t_start) * 1000;

if sol_qp.problem == 0
    qp_status = 0;
    fallback_active = 0;
    dU_opt = value(dU_var);
    u_nom_cmd = u_xi_prev + dU_opt(1);
    % Clamp to physical limits
    u_nom_cmd = max(u_min, min(u_max, u_nom_cmd));
else
    qp_status = 1;
    fallback_active = 1;
    % Fallback: slew-limited step relative to previous applied control
    if a_p < -0.1 || x_nom(2) < -0.1
        u_nom_cmd = max(u_min, u_app_prev + delta_u_min);
    else
        u_nom_cmd = max(u_min, min(u_max, u_app_prev));
    end
end

% Downstream compensation:
% Eq27_Law adds K*e_xi - d_hat. Since Controller A has K = [0 0 0],
% setting u_xi_cmd = u_nom_cmd + d_hat ensures u_raw = (u_nom_cmd + d_hat) - d_hat = u_nom_cmd!
u_xi_cmd = u_nom_cmd + d_hat;

min_w = u_max - u_min; % 5.50 m/s^2 (constant un-tightened)
peak_sig = 0.0;        % No uncertainty propagation

out = [u_xi_cmd; qp_status; fallback_active; min_w; peak_sig; solve_time_ms];
end
