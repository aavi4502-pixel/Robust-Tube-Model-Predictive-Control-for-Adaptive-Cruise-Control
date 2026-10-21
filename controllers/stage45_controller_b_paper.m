function out = stage45_controller_b_paper(in_vec)
% STAGE45_CONTROLLER_B_PAPER
% Controller B: Original Paper-Style Static Uncertainty-Bound Robust MPC Controller.
%
% Uses static worst-case uncertainty propagation (u_bar = 3.0 m/s^2 at all steps),
% legacy tube gain K = [2.4855, -0.2200, -2.0959], gu = 0.833350, sigma* = 10.8896.
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
    
    c_const = load('stage43e_candidate_constraints.mat', 'Hx_corr', 'hx_corr', 'gx_corr');
    c_qp    = load('stage43a_qp_matrices.mat', 'Phi', 'Gamma', 'Q_bar', 'R_bar', 'W_bar', ...
                   'delta_u_max', 'delta_u_min');
    c_lft   = load('stage42h3_lft_scenario.mat', 'theta1', 'theta2', 'theta3');
    c_base  = load('baseline_provisional_r10_params.mat', 'A_c', 'B_c', 'Ts', 'K_tube');
    
    N = 6;
    precomp.N = N;
    precomp.Ts = c_base.Ts;
    precomp.K_tube = c_base.K_tube; % [2.4855, -0.2200, -2.0959]
    precomp.gu = 0.833350;          % Paper static tightening
    precomp.gx_corr = c_const.gx_corr; % [0.3944; 0.3944; 0.2538; 0.2538; 0.7923; 0.7923]
    precomp.sigma_star = 10.8896;
    precomp.gamma = 1.0000;
    precomp.rho_sq = 0.95^2;
    precomp.ed_max = 0.05;
    precomp.delta_a_bar = 0.20;
    precomp.u_max = 2.50;
    precomp.u_min = -3.00;
    precomp.u_bar_peak = 3.00;      % Static physical peak
    precomp.delta_u_max = c_qp.delta_u_max; % 0.25
    precomp.delta_u_min = c_qp.delta_u_min; % -0.25
    precomp.theta1 = c_lft.theta1;
    precomp.theta2 = c_lft.theta2;
    precomp.theta3 = c_lft.theta3;
    
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
    
    nHx = 6;
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
    precomp.nHx = nHx;
end

x_nom      = in_vec(1:3);
x_hat      = in_vec(4:6);
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
Ts = precomp.Ts;
theta1 = precomp.theta1;
theta2 = precomp.theta2;
theta3 = precomp.theta3;
u_max = precomp.u_max;
u_min = precomp.u_min;
delta_u_max = precomp.delta_u_max;
delta_u_min = precomp.delta_u_min;
gu = precomp.gu;
gamma = precomp.gamma;
rho_sq = precomp.rho_sq;
ed_max = precomp.ed_max;
delta_a_bar = precomp.delta_a_bar;
u_bar_peak = precomp.u_bar_peak;

% Ancillary compensation term c_t
e_xi = x_hat - x_nom;
K_exi = precomp.K_tube * e_xi;
c_t = K_exi - d_hat;

% Static paper-style uncertainty propagation using u_bar_peak = 3.0 m/s^2
v_0 = abs(x_nom(2));
a_0 = abs(x_nom(3));
z_static = zeros(1, N);
for k = 0:(N-1)
    z_static(k+1) = theta1*v_0 + (k*Ts*theta1 + theta2)*a_0 ...
                  + (k*(k-1)/2 * Ts*theta1 + k*theta2)*delta_a_bar ...
                  + theta3 * u_bar_peak; % Constant worst-case peak
end

sig_static = zeros(1, N+1);
for k = 0:(N-1)
    sig_static(k+2) = rho_sq * sig_static(k+1) + gamma*(ed_max^2 + z_static(k+1)^2);
end
peak_sig = max(sig_static);

% Tightened input intervals
u_max_t = u_max - gu * sqrt(sig_static(1:N))';
u_min_t = u_min + gu * sqrt(sig_static(1:N))';
w_t = u_max_t - u_min_t;
min_w = min(w_t);

% Check interval collapse
if any(w_t < 0)
    is_feas = false;
    dU_opt = zeros(N, 1);
else
    % Formulate tightened QP
    Ap_vec = a_p * ones(N, 1);
    f_xi = precomp.S_x * x_nom + precomp.S_u * u_xi_prev + precomp.S_ap * Ap_vec;
    c_qp = 2 * (precomp.G_delta' * precomp.Q_bar * f_xi + precomp.L_delta' * precomp.R_bar * (ones(N, 1) * u_xi_prev));
    
    b_slew  = [ delta_u_max * ones(N, 1); -delta_u_min * ones(N, 1) ];
    b_slew(1)   = min(delta_u_max, u_max_t(1) - u_xi_prev);
    b_slew(N+1) = min(-delta_u_min, -(u_min_t(1) - u_xi_prev));
    
    b_input = [ u_max_t - ones(N, 1)*u_xi_prev + ones(N, 1)*d_hat;
               -u_min_t + ones(N, 1)*u_xi_prev - ones(N, 1)*d_hat ];
           
    hx_bar_tight = zeros(precomp.nHx * N, 1);
    for k = 1:N
        hx_bar_tight((k-1)*precomp.nHx + 1 : k*precomp.nHx) = precomp.hx_corr - precomp.gx_corr * sqrt(sig_static(k+1));
    end
    b_state = hx_bar_tight - precomp.Hx_bar * f_xi;
    b_tight = [ b_slew; b_input; b_state ];
    
    yalmip('clear');
    dU_var = sdpvar(N, 1);
    ops_silent = sdpsettings('solver', 'sdpt3', 'verbose', 0);
    sol_qp = optimize([ precomp.A_ineq * dU_var <= b_tight ], ...
                      0.5 * dU_var' * precomp.H_qp * dU_var + c_qp' * dU_var, ops_silent);
                  
    if sol_qp.problem == 0
        is_feas = true;
        dU_opt = value(dU_var);
    else
        is_feas = false;
        dU_opt = zeros(N, 1);
    end
end

solve_time_ms = toc(t_start) * 1000;

if is_feas
    qp_status = 0;
    fallback_active = 0;
    u_xi_cmd = u_xi_prev + dU_opt(1);
    u_xi_cmd = max(u_min_t(1), min(u_max_t(1), u_xi_cmd));
else
    qp_status = 1;
    fallback_active = 1;
    % Legacy fallback
    if a_p < -0.1 || x_nom(2) < -0.1
        u_xi_cmd = max(u_min, u_xi_prev + delta_u_min);
    else
        u_xi_cmd = max(u_min, min(u_max, u_xi_prev));
    end
end

out = [u_xi_cmd; qp_status; fallback_active; min_w; peak_sig; solve_time_ms];
end
