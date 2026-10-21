function out = stage46_ablation_controller(in_vec, variant_id)
% STAGE46_ABLATION_CONTROLLER
% Parametric Controller for Stage 46B Ablation Study.
% Evaluates 5 orthogonal variants under identical standardized fallback:
%
%   Variant A: Static Bound,      Ancillary Step-0: No,   Gain: Fixed (K_paper)
%   Variant B: Trajectory Bound,  Ancillary Step-0: No,   Gain: Fixed (K_paper)
%   Variant C: Static Bound,      Ancillary Step-0: Yes,  Gain: Fixed (K_paper)
%   Variant D: Trajectory Bound,  Ancillary Step-0: Yes,  Gain: Fixed (K_paper)
%   Variant E: Trajectory Bound,  Ancillary Step-0: Yes,  Gain: Co-Designed (K_opt)
%
% All variants share:
%   - Prediction horizon N = 6, Ts = 0.1s
%   - Cost matrices Q_bar, R_bar, W_bar
%   - Physical bounds: u in [-3.0, 2.5] m/s^2, |du| <= 0.25 m/s^2
%   - Standardized slew-invariant fallback policy: u_target = max(u_min, u_app_prev + delta_u_min)

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
    c_lmi   = load('stage44c_lmi_candidate.mat', 'lmi_results');
    cand2   = c_lmi.lmi_results(2); % Co-designed candidate K = [1.2345, -0.5985, -1.2588]
    
    N = 6;
    precomp.N = N;
    precomp.Ts = c_base.Ts;
    
    % Fixed paper parameters
    precomp.K_paper      = c_base.K_tube; % [2.4855, -0.2200, -2.0959]
    precomp.gu_paper     = 0.833350;
    precomp.gx_paper     = c_const.gx_corr; % [0.3944; 0.3944; 0.2538; 0.2538; 0.7923; 0.7923]
    precomp.sig_star_p   = 10.8896;
    precomp.rho_sq_paper = 0.95^2;
    
    % Co-designed parameters
    precomp.K_opt        = cand2.K; % [1.2345, -0.5985, -1.2588]
    precomp.gu_opt       = cand2.gu; % 0.755533
    precomp.gx_opt       = cand2.gx;
    precomp.sig_star_opt = cand2.sigma_star; % 13.2482
    precomp.rho_sq_opt   = cand2.rho_K^2; % (0.9560)^2
    
    precomp.gamma       = 1.0000;
    precomp.ed_max      = 0.05;
    if evalin('base', 'exist(''override_delta_a_bar'', ''var'')')
        precomp.delta_a_bar = evalin('base', 'override_delta_a_bar');
    else
        precomp.delta_a_bar = 0.20;
    end
    precomp.u_max       = 2.50;
    precomp.u_min       = -3.00;
    precomp.u_bar_peak  = 3.00;
    precomp.delta_u_max = c_qp.delta_u_max; % 0.25 m/s^2
    precomp.delta_u_min = c_qp.delta_u_min; % -0.25 m/s^2
    precomp.theta1      = c_lft.theta1;
    precomp.theta2      = c_lft.theta2;
    precomp.theta3      = c_lft.theta3;
    
    % Discretization of lead acceleration channel
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

% Determine variant configuration
% Normalize string
if ischar(variant_id) || isstring(variant_id)
    v_str = char(variant_id);
    if startsWith(v_str, 'ABL_')
        v_key = v_str(5:end);
    else
        v_key = v_str;
    end
else
    v_key = 'E';
end

switch upper(v_key)
    case 'A'
        use_traj_bound = false;
        use_ancillary_step0 = false;
        K_tube = precomp.K_paper;
        gu = precomp.gu_paper;
        gx = precomp.gx_paper;
        rho_sq = precomp.rho_sq_paper;
    case 'B'
        use_traj_bound = true;
        use_ancillary_step0 = false;
        K_tube = precomp.K_paper;
        gu = precomp.gu_paper;
        gx = precomp.gx_paper;
        rho_sq = precomp.rho_sq_paper;
    case 'C'
        use_traj_bound = false;
        use_ancillary_step0 = true;
        K_tube = precomp.K_paper;
        gu = precomp.gu_paper;
        gx = precomp.gx_paper;
        rho_sq = precomp.rho_sq_paper;
    case 'D'
        use_traj_bound = true;
        use_ancillary_step0 = true;
        K_tube = precomp.K_paper;
        gu = precomp.gu_paper;
        gx = precomp.gx_paper;
        rho_sq = precomp.rho_sq_paper;
    case 'E'
        use_traj_bound = true;
        use_ancillary_step0 = true;
        K_tube = precomp.K_opt;
        gu = precomp.gu_opt;
        gx = precomp.gx_opt;
        rho_sq = precomp.rho_sq_opt;
    otherwise
        error('Unknown ablation variant: %s', v_key);
end

N = precomp.N;
Ts = precomp.Ts;
theta1 = precomp.theta1;
theta2 = precomp.theta2;
theta3 = precomp.theta3;
u_max = precomp.u_max;
u_min = precomp.u_min;
delta_u_max = precomp.delta_u_max;
delta_u_min = precomp.delta_u_min;
gamma = precomp.gamma;
ed_max = precomp.ed_max;
delta_a_bar = precomp.delta_a_bar;

% 1. Compute exact instantaneous ancillary correction c_t
e_xi = x_hat - x_nom;
K_exi = K_tube * e_xi;
c_t = K_exi - d_hat;

% Step-0 bounds setup
if use_ancillary_step0
    u_xi0_LB = max([ u_min - c_t, ...
                     u_app_prev + delta_u_min - c_t, ...
                     u_xi_prev + delta_u_min, ...
                     u_min + d_hat ]);
    u_xi0_UB = min([ u_max - c_t, ...
                     u_app_prev + delta_u_max - c_t, ...
                     u_xi_prev + delta_u_max, ...
                     u_max + d_hat ]);
    w_0 = u_xi0_UB - u_xi0_LB;
else
    % Standard nominal bounds (no ancillary awareness)
    u_xi0_LB = u_min + d_hat;
    u_xi0_UB = u_max + d_hat;
    w_0 = u_max - u_min;
end

% Prediction vectors
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

% Early exit if step-0 bounds already collapsed
if use_ancillary_step0 && w_0 < 0
    is_feas = false;
    dU_opt = zeros(N, 1);
    min_w = w_0;
    max_sig = 0;
else
    if ~use_traj_bound
        % -------------------------------------------------------------
        % STATIC UNCERTAINTY PROPAGATION (u_bar_peak = 3.0 m/s^2)
        % -------------------------------------------------------------
        z_static = z_kin + theta3 * precomp.u_bar_peak;
        sig_static = zeros(1, N+1);
        for k = 0:(N-1)
            sig_static(k+2) = rho_sq * sig_static(k+1) + gamma*(ed_max^2 + z_static(k+1)^2);
        end
        max_sig = max(sig_static);
        
        u_max_t = u_max - gu * sqrt(sig_static(1:N))';
        u_min_t = u_min + gu * sqrt(sig_static(1:N))';
        w_t = u_max_t - u_min_t;
        if use_ancillary_step0
            w_t(1) = w_0;
        end
        min_w = min(w_t);
        
        if any(w_t < 0)
            is_feas = false;
            dU_opt = zeros(N, 1);
        else
            b_slew  = [ delta_u_max * ones(N, 1); -delta_u_min * ones(N, 1) ];
            if use_ancillary_step0
                b_slew(1)   = min(delta_u_max, u_xi0_UB - u_xi_prev);
                b_slew(N+1) = min(-delta_u_min, -(u_xi0_LB - u_xi_prev));
            else
                b_slew(1)   = min(delta_u_max, u_max_t(1) - u_xi_prev);
                b_slew(N+1) = min(-delta_u_min, -(u_min_t(1) - u_xi_prev));
            end
            
            b_input = [ u_max_t - ones(N, 1)*u_xi_prev + ones(N, 1)*d_hat;
                       -u_min_t + ones(N, 1)*u_xi_prev - ones(N, 1)*d_hat ];
            if use_ancillary_step0
                b_input(1)   = u_xi0_UB - u_xi_prev;
                b_input(N+1) = -(u_xi0_LB - u_xi_prev);
            end
            
            hx_bar_tight = zeros(precomp.nHx * N, 1);
            for k = 1:N
                hx_bar_tight((k-1)*precomp.nHx + 1 : k*precomp.nHx) = precomp.hx_corr - gx * sqrt(sig_static(k+1));
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
    else
        % -------------------------------------------------------------
        % TRAJECTORY-DEPENDENT UNCERTAINTY PROPAGATION (Sequential QP)
        % -------------------------------------------------------------
        % Warm start via un-tightened nominal QP
        yalmip('clear');
        dU_nom = sdpvar(N, 1);
        ops_silent = sdpsettings('solver', 'sdpt3', 'verbose', 0);
        
        b_slew_nom = [ delta_u_max * ones(N, 1); -delta_u_min * ones(N, 1) ];
        if use_ancillary_step0
            b_slew_nom(1)   = min(delta_u_max, u_xi0_UB - u_xi_prev);
            b_slew_nom(N+1) = min(-delta_u_min, -(u_xi0_LB - u_xi_prev));
            b_u_nom = [ u_max * ones(N, 1) - ones(N, 1)*u_xi_prev + ones(N, 1)*d_hat;
                       -u_min * ones(N, 1) + ones(N, 1)*u_xi_prev - ones(N, 1)*d_hat ];
            b_u_nom(1)   = u_xi0_UB - u_xi_prev;
            b_u_nom(N+1) = -(u_xi0_LB - u_xi_prev);
        else
            b_slew_nom(1)   = min(delta_u_max, u_max - u_xi_prev);
            b_slew_nom(N+1) = min(-delta_u_min, -(u_min - u_xi_prev));
            b_u_nom = [ u_max * ones(N, 1) - ones(N, 1)*u_xi_prev + ones(N, 1)*d_hat;
                       -u_min * ones(N, 1) + ones(N, 1)*u_xi_prev - ones(N, 1)*d_hat ];
        end
        b_nom = [ b_slew_nom; b_u_nom; kron(ones(N, 1), precomp.hx_corr) - precomp.Hx_bar * f_xi ];
        sol_nom = optimize([ precomp.A_ineq * dU_nom <= b_nom ], ...
                           0.5 * dU_nom' * precomp.H_qp * dU_nom + c_qp' * dU_nom, ops_silent);
                       
        if sol_nom.problem == 0
            U_curr = ones(N, 1)*u_xi_prev + precomp.C_u_bar * value(dU_nom);
        else
            U_curr = ones(N, 1) * u_xi_prev;
        end
        
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
            if use_ancillary_step0
                w_t(1) = w_0;
            end
            min_w = min(w_t);
            
            if any(w_t < 0)
                is_feas = false;
                break;
            end
            
            b_slew  = [ delta_u_max * ones(N, 1); -delta_u_min * ones(N, 1) ];
            if use_ancillary_step0
                b_slew(1)   = min(delta_u_max, u_xi0_UB - u_xi_prev);
                b_slew(N+1) = min(-delta_u_min, -(u_xi0_LB - u_xi_prev));
            else
                b_slew(1)   = min(delta_u_max, u_max_t(1) - u_xi_prev);
                b_slew(N+1) = min(-delta_u_min, -(u_min_t(1) - u_xi_prev));
            end
            
            b_input = [ u_max_t - ones(N, 1)*u_xi_prev + ones(N, 1)*d_hat;
                       -u_min_t + ones(N, 1)*u_xi_prev - ones(N, 1)*d_hat ];
            if use_ancillary_step0
                b_input(1)   = u_xi0_UB - u_xi_prev;
                b_input(N+1) = -(u_xi0_LB - u_xi_prev);
            end
            
            hx_bar_tight = zeros(precomp.nHx * N, 1);
            for k = 1:N
                hx_bar_tight((k-1)*precomp.nHx + 1 : k*precomp.nHx) = precomp.hx_corr - gx * sqrt(sig_traj(k+1));
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
end

solve_time_ms = toc(t_start) * 1000;

% Control Command & Standardized Fallback
if is_feas
    qp_status = 0;
    fallback_active = 0;
    u_xi_cmd = u_xi_prev + dU_opt(1);
    if use_ancillary_step0
        u_xi_cmd = max(u_xi0_LB, min(u_xi0_UB, u_xi_cmd));
    else
        u_xi_cmd = max(u_min_t(1), min(u_max_t(1), u_xi_cmd));
    end
else
    qp_status = 1;
    fallback_active = 1;
    
    % STANDARDIZED SLEW-INVARIANT FALLBACK POLICY ACROSS ALL VARIANTS
    % Deceleration schedule based on previous applied control
    u_target_min = max(u_min, u_app_prev + delta_u_min); % u_app_prev - 0.25
    u_target_max = min(u_max, u_app_prev + delta_u_max); % u_app_prev + 0.25
    
    if a_p < -0.1 || x_nom(2) < -0.1
        u_target = u_target_min; % Maximum allowable braking step
    else
        u_target = max(u_target_min, min(u_target_max, u_app_prev));
    end
    
    % Enforce downstream u_raw = u_target exactly:
    % Since u_raw = u_xi + c_t, setting u_xi_cmd = u_target - c_t guarantees u_raw == u_target!
    u_xi_cmd = u_target - c_t;
end

out = [u_xi_cmd; qp_status; fallback_active; min_w; max_sig; solve_time_ms];
end
