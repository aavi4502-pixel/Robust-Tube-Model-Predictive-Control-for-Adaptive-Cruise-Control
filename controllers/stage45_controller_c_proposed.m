function out = stage45_controller_c_proposed(in_vec)
% STAGE45_CONTROLLER_C_PROPOSED
% Controller C: Proposed Co-Designed Trajectory-Coupled Robust MPC with
% Instantaneous Ancillary Tightening and Slew-Invariant Fallback.
%
% Strictly delegates to validated stage44c_mpc_solver.

out = stage44c_mpc_solver(in_vec);
end
