function out = stage45_mpc_dispatcher(in_vec)
% STAGE45_MPC_DISPATCHER
% Modular dispatcher for Stage 45 Tri-Partite Comparison.
%
% Dispatches in_vec to:
%   'A' -> stage45_controller_a_nominal(in_vec)
%   'B' -> stage45_controller_b_paper(in_vec)
%   'C' -> stage45_controller_c_proposed(in_vec)

persistent active_ctrl;

try
    active_ctrl = evalin('base', 'stage45_active_controller');
catch
    if isempty(active_ctrl)
        active_ctrl = 'C';
    end
end

switch active_ctrl
    case 'A'
        out = stage45_controller_a_nominal(in_vec);
    case 'B'
        out = stage45_controller_b_paper(in_vec);
    case 'C'
        out = stage45_controller_c_proposed(in_vec);
    case {'ABL_A', 'ABL_B', 'ABL_C', 'ABL_D', 'ABL_E'}
        out = stage46_ablation_controller(in_vec, active_ctrl);
    otherwise
        error('Unknown active controller: %s', active_ctrl);
end

end
