function out = stage44c_mpc_solver(in_vec)
% STAGE44C_MPC_SOLVER
% Stage 44C: Co-Designed Trajectory-Dependent Sequential QP Controller with
% Exact Fallback Slew-Rate Invariance and LMI-Synthesized Tube Parameters.
%
% Input vector in_vec:
%   in_vec(1:3)  : Nominal state z_xi = [delta_d; delta_v; a]
%   in_vec(4:6)  : Estimated state x_hat = [delta_d; delta_v; a]
%   in_vec(7)    : Estimated disturbance d_hat
%   in_vec(8)    : Previous nominal control u_xi_prev
%   in_vec(9)    : Lead vehicle acceleration a_p
%   in_vec(10)   : Current simulation time t
%   in_vec(11)   : Previous applied control u_app_prev (from ActuatorSat)
%
% Output vector out:
%   out(1) : Commanded nominal control u_xi_cmd
%   out(2) : QP status (0 = Feasible, 1 = Infeasible)
%   out(3) : Fallback active (0 = Normal, 1 = Fallback)
%   out(4) : Minimum input interval span w_min [m/s^2]
%   out(5) : Peak variance bound sigma_max
%   out(6) : Solver execution time [ms]

persistent precomp fallback_log;

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
    
    % Load validated candidate parameters and co-designed LMI data
    c_const = load('stage43e_candidate_constraints.mat', 'Hx_corr', 'hx_corr');
    c_qp    = load('stage43a_qp_matrices.mat', 'Phi', 'Gamma', 'Q_bar', 'R_bar', 'W_bar', ...
                   'delta_u_max', 'delta_u_min');
    c_lft   = load('stage42h3_lft_scenario.mat', 'theta1', 'theta2', 'theta3');
    c_base  = load('baseline_provisional_r10_params.mat', 'A_c', 'B_c', 'Ts');
    
    % Load Stage 44C co-designed LMI candidate (Candidate 2)
    c_lmi = load('stage44c_lmi_candidate.mat', 'lmi_results');
    cand2 = c_lmi.lmi_results(2); % Candidate 2 (R=50): K = [1.2345, -0.5985, -1.2588]
    
    N = 6; % Validated horizon
    precomp.N = N;
    precomp.Ts = c_base.Ts;
    precomp.K_tube = cand2.K;
    precomp.gu = cand2.gu;          % 0.755533
    precomp.gx_corr = cand2.gx;     % Co-designed state tightening bounds
    precomp.sigma_star = cand2.sigma_star; % 13.2482
    precomp.gamma = cand2.gamma;    % 1.0000
    precomp.rho_sq = cand2.rho_K^2; % (0.9560)^2
    precomp.ed_max = 0.05;
    precomp.delta_a_bar = 0.20;
    precomp.u_max = 2.50;
    precomp.u_min = -3.00;
    precomp.delta_u_max = c_qp.delta_u_max; % 0.25 m/s^2
    precomp.delta_u_min = c_qp.delta_u_min; % -0.25 m/s^2
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
    
    % Prediction matrices
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
    precomp.Hx_corr = c_const.Hx_corr;
    precomp.hx_corr = c_const.hx_corr;
    precomp.Hx_bar = Hx_bar_N;
    precomp.A_ineq = A_ineq_N;
    precomp.nHx = nHx;
    
    fallback_log = [];
end

% Parse inputs
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
K_tube = precomp.K_tube;

% 1. Compute exact instantaneous ancillary correction c_t
e_xi = x_hat - x_nom;
K_exi = K_tube * e_xi;
c_t = K_exi - d_hat;

% Stage 44C: Exact instantaneous bounds on first nominal input u_xi,0|t
u_xi0_LB = max([ u_min - c_t, ...
                 u_app_prev + delta_u_min - c_t, ...
                 u_xi_prev + delta_u_min, ...
                 u_min + d_hat ]);
             
u_xi0_UB = min([ u_max - c_t, ...
                 u_app_prev + delta_u_max - c_t, ...
                 u_xi_prev + delta_u_max, ...
                 u_max + d_hat ]);

w_0 = u_xi0_UB - u_xi0_LB;

% Prediction vector and linear cost
Ap_vec = a_p * ones(N, 1);
f_xi = precomp.S_x * x_nom + precomp.S_u * u_xi_prev + precomp.S_ap * Ap_vec;
c_qp = 2 * (precomp.G_delta' * precomp.Q_bar * f_xi + precomp.L_delta' * precomp.R_bar * (ones(N, 1) * u_xi_prev));

% Kinematic uncertainty term
v_0 = abs(x_nom(2));
a_0 = abs(x_nom(3));
z_kin = zeros(1, N);
for k = 0:(N-1)
    z_kin(k+1) = theta1*v_0 + (k*Ts*theta1 + theta2)*a_0 ...
               + (k*(k-1)/2 * Ts*theta1 + k*theta2)*delta_a_bar;
end

% Check if step-0 interval is already collapsed
if w_0 < 0
    is_feas = false;
    dU_opt = zeros(N, 1);
    min_w = w_0;
    max_sig = 0;
else
    % 2. Warm-start via nominal un-tightened solve incorporating step-0 bounds
    yalmip('clear');
    dU_nom = sdpvar(N, 1);
    
    b_slew_nom = [ delta_u_max * ones(N, 1); -delta_u_min * ones(N, 1) ];
    b_slew_nom(1)   = min(delta_u_max, u_xi0_UB - u_xi_prev);
    b_slew_nom(N+1) = min(-delta_u_min, -(u_xi0_LB - u_xi_prev));
    
    b_u_nom = [ u_max * ones(N, 1) - ones(N, 1)*u_xi_prev + ones(N, 1)*d_hat;
               -u_min * ones(N, 1) + ones(N, 1)*u_xi_prev - ones(N, 1)*d_hat ];
    b_u_nom(1)   = u_xi0_UB - u_xi_prev;
    b_u_nom(N+1) = -(u_xi0_LB - u_xi_prev);
    
    b_nom = [ b_slew_nom; b_u_nom; kron(ones(N, 1), precomp.hx_corr) - precomp.Hx_bar * f_xi ];
    ops_silent = sdpsettings('solver', 'sdpt3', 'verbose', 0);
    sol_nom = optimize([ precomp.A_ineq * dU_nom <= b_nom ], ...
                       0.5 * dU_nom' * precomp.H_qp * dU_nom + c_qp' * dU_nom, ops_silent);
    
    if sol_nom.problem == 0
        U_curr = ones(N, 1)*u_xi_prev + precomp.C_u_bar * value(dU_nom);
    else
        U_curr = ones(N, 1) * u_xi_prev;
    end
    
    % 3. Sequential tightening evaluation incorporating step-0 exact bounds
    max_iter = 3;
    tol_conv = 1e-4;
    is_feas = false;
    dU_opt = zeros(N, 1);
    min_w = w_0;
    max_sig = 0;
    
    for it = 1:max_iter
        z_traj = z_kin + theta3 * abs(U_curr)';
        sig_traj = zeros(1, N+1);
        for k = 0:(N-1)
            sig_traj(k+2) = rho_sq * sig_traj(k+1) + gamma*(ed_max^2 + z_traj(k+1)^2);
        end
        max_sig = max(sig_traj);
        
        u_max_t = u_max - gu * sqrt(sig_traj(1:N))';
        u_min_t = u_min + gu * sqrt(sig_traj(1:N))';
        w_t = u_max_t - u_min_t;
        w_t(1) = w_0; % Step 0 exact bound
        min_w = min(w_t);
        
        if any(w_t < 0)
            is_feas = false;
            break;
        end
        
        b_slew  = [ delta_u_max * ones(N, 1); -delta_u_min * ones(N, 1) ];
        b_slew(1)   = min(delta_u_max, u_xi0_UB - u_xi_prev);
        b_slew(N+1) = min(-delta_u_min, -(u_xi0_LB - u_xi_prev));
        
        b_input = [ u_max_t - ones(N, 1)*u_xi_prev + ones(N, 1)*d_hat;
                   -u_min_t + ones(N, 1)*u_xi_prev - ones(N, 1)*d_hat ];
        b_input(1)   = u_xi0_UB - u_xi_prev;
        b_input(N+1) = -(u_xi0_LB - u_xi_prev);
        
        hx_bar_tight = zeros(precomp.nHx * N, 1);
        for k = 1:N
            hx_bar_tight((k-1)*precomp.nHx + 1 : k*precomp.nHx) = precomp.hx_corr - precomp.gx_corr * sqrt(sig_traj(k+1));
        end
        b_state = hx_bar_tight - precomp.Hx_bar * f_xi;
        b_tight = [ b_slew; b_input; b_state ];
        
        yalmip('clear');
        dU_var = sdpvar(N, 1);
        sol_qp = optimize([ precomp.A_ineq * dU_var <= b_tight ], ...
                          0.5 * dU_var' * precomp.H_qp * dU_var + c_qp' * dU_var, ops_silent);
                      
        if sol_qp.problem == 0
            dU_cand = value(dU_var);
            U_cand  = ones(N, 1)*u_xi_prev + precomp.C_u_bar * dU_cand;
            res = max(abs(U_cand - U_curr));
            dU_opt = dU_cand;
            is_feas = true;
            if res <= tol_conv
                break;
            end
            U_curr = U_cand;
        else
            is_feas = false;
            break;
        end
    end
end

solve_time_ms = toc(t_start) * 1000;

% 4. Control Command & Fallback Handling with Exact Slew Invariance
if is_feas
    qp_status = 0;
    fallback_active = 0;
    u_xi_cmd = u_xi_prev + dU_opt(1);
    % Hard clamp to allowable bounds
    u_xi_cmd = max(u_xi0_LB, min(u_xi0_UB, u_xi_cmd));
else
    qp_status = 1;
    fallback_active = 1;
    
    % Stage 44C Task 1: Bulletproof Slew-Rate and Actuator Invariance
    % Enforce bounds directly against previous applied control u_app_prev!
    u_target_min = max(u_min, u_app_prev + delta_u_min); % u_app_prev - 0.25
    u_target_max = min(u_max, u_app_prev + delta_u_max); % u_app_prev + 0.25
    
    % Deceleration policy under fallback
    if a_p < -0.1 || x_nom(2) < -0.1
        u_target = u_target_min; % Maximum allowable braking step
    else
        u_target = max(u_target_min, min(u_target_max, u_app_prev));
    end
    
    % Enforce u_raw = u_target exactly downstream:
    % Since u_raw = u_xi + c_t, setting u_xi_cmd = u_target - c_t guarantees u_raw == u_target!
    u_xi_cmd = u_target - c_t;
    
    % Record explicit fallback log entry
    log_entry = [t, x_nom(1), x_nom(2), x_nom(3), u_xi_prev, u_app_prev, c_t, w_0, min_w, max_sig];
    fallback_log = [fallback_log; log_entry];
    assignin('base', 'stage44c_fallback_log', fallback_log);
end

out = [u_xi_cmd; qp_status; fallback_active; min_w; max_sig; solve_time_ms];
end
