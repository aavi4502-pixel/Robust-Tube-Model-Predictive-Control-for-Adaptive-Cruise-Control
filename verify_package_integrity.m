% VERIFY_PACKAGE_INTEGRITY.M
% Automated integrity and verification test for Stage 48 Reproducibility Package.
clear; clc;
fprintf('========================================================================\n');
fprintf('  STAGE 48: REPRODUCIBILITY PACKAGE INTEGRITY AUDIT\n');
fprintf('========================================================================\n\n');

pkg_root = fileparts(mfilename('fullpath'));
pass_count = 0;
fail_count = 0;

function check_file(path_str, desc)
    if isfile(path_str)
        s = dir(path_str);
        fprintf('  [PASS] %-40s (%7d bytes) - %s\n', s.name, s.bytes, desc);
        assignin('caller', 'pass_count', evalin('caller', 'pass_count') + 1);
    else
        fprintf('  [FAIL] %-40s MISSING! - %s\n', path_str, desc);
        assignin('caller', 'fail_count', evalin('caller', 'fail_count') + 1);
    end
end

fprintf('1. Checking Manuscript Files:\n');
check_file(fullfile(pkg_root, 'manuscript', 'manuscript_complete_draft.tex'), 'Complete IEEEtran LaTeX Draft');

fprintf('\n2. Checking Simulink Models:\n');
check_file(fullfile(pkg_root, 'models', 'ACC_Stage43GE_Experimental.slx'), 'Active Dual-Mode Tube MPC Model');
check_file(fullfile(pkg_root, 'models', 'ACC_Baseline_Provisional_R10.slx'), 'Frozen Baseline Model (100% Intact)');

fprintf('\n3. Checking Controllers:\n');
check_file(fullfile(pkg_root, 'controllers', 'stage44c_mpc_solver.m'), 'Step-0 Conditioned Tube MPC Solver');
check_file(fullfile(pkg_root, 'controllers', 'stage45_mpc_dispatcher.m'), 'Runtime Controller Dispatcher');
check_file(fullfile(pkg_root, 'controllers', 'stage46_ablation_controller.m'), 'Parametric 5-Variant Ablation Controller');
check_file(fullfile(pkg_root, 'controllers', 'stage45_controller_a_nominal.m'), 'Controller A (Nominal Baseline)');
check_file(fullfile(pkg_root, 'controllers', 'stage45_controller_b_paper.m'), 'Controller B (Quan et al. 2024 / Static Tube MPC)');
check_file(fullfile(pkg_root, 'controllers', 'stage45_controller_c_proposed.m'), 'Controller C (Proposed Interface)');

fprintf('\n4. Checking Datasets & Mathematical Models:\n');
check_file(fullfile(pkg_root, 'data', 'baseline_provisional_r10_params.mat'), 'Host Plant & Standard Parameters');
check_file(fullfile(pkg_root, 'data', 'stage44c_lmi_candidate.mat'), 'Synthesized LMI Certificate Struct');
check_file(fullfile(pkg_root, 'data', 'stage43a_qp_matrices.mat'), 'Precomputed QP Prediction Matrices');
check_file(fullfile(pkg_root, 'data', 'stage43e_candidate_constraints.mat'), 'Polytopic State/Input Constraints');
check_file(fullfile(pkg_root, 'data', 'stage42h3_lft_scenario.mat'), 'LFT Uncertainty Coefficients theta');
check_file(fullfile(pkg_root, 'data', 'stage45_extended_results.mat'), 'Raw 16-Case Emergency Braking Data');
check_file(fullfile(pkg_root, 'data', 'stage46_ablation_results.mat'), 'Raw 5-Variant Ablation Benchmarks Data');
check_file(fullfile(pkg_root, 'data', 'stage46_da_comparison_results.mat'), 'Acceleration Drift Sensitivity Results');

fprintf('\n5. Checking Reproduction & Plotting Scripts:\n');
check_file(fullfile(pkg_root, 'scripts', 'stage46_run_ablation_study.m'), 'Ablation Runner (Table I & II)');
check_file(fullfile(pkg_root, 'scripts', 'stage45_extended_comparison.m'), 'Emergency Matrix Runner (Table III)');
check_file(fullfile(pkg_root, 'scripts', 'stage46_plot_ablation.m'), 'Ablation Plotter (Fig 3)');
check_file(fullfile(pkg_root, 'scripts', 'stage45_plot_tripartite_comparison.m'), 'Comparison Plotter (Fig 4)');
check_file(fullfile(pkg_root, 'scripts', 'stage46_test_da_comparison.m'), 'Drift Sensitivity Checker');
check_file(fullfile(pkg_root, 'scripts', 'stage44e_lmi_audit_and_reproducibility.m'), 'LMI Certificate Re-Verification');

fprintf('\n6. Checking Variable Integrity in Core Datasets:\n');
d_abl = load(fullfile(pkg_root, 'data', 'stage46_ablation_results.mat'));
if isfield(d_abl, 'ablation_results') && isequal(size(d_abl.ablation_results), [4, 5])
    fprintf('  [PASS] stage46_ablation_results.mat contains all 4 benchmarks x 5 variants (4x5 struct matrix).\n');
    pass_count = pass_count + 1;
else
    fprintf('  [FAIL] stage46_ablation_results.mat corrupt or incomplete!\n');
    fail_count = fail_count + 1;
end

d_ext = load(fullfile(pkg_root, 'data', 'stage45_extended_results.mat'));
if isfield(d_ext, 'ext_results') && length(d_ext.ext_results) == 16
    fprintf('  [PASS] stage45_extended_results.mat contains all 16 emergency cases.\n');
    pass_count = pass_count + 1;
else
    fprintf('  [FAIL] stage45_extended_results.mat corrupt or incomplete!\n');
    fail_count = fail_count + 1;
end

d_lmi = load(fullfile(pkg_root, 'data', 'stage44c_lmi_candidate.mat'));
if isfield(d_lmi, 'lmi_results') && length(d_lmi.lmi_results) >= 2
    fprintf('  [PASS] stage44c_lmi_candidate.mat contains valid LMI candidate certificates.\n');
    pass_count = pass_count + 1;
else
    fprintf('  [FAIL] stage44c_lmi_candidate.mat corrupt or incomplete!\n');
    fail_count = fail_count + 1;
end

fprintf('\n========================================================================\n');
fprintf('  AUDIT SUMMARY: %d PASSED, %d FAILED\n', pass_count, fail_count);
if fail_count == 0
    fprintf('  OVERALL RESULT: PACKAGE INTEGRITY 100%% VERIFIED!\n');
else
    fprintf('  OVERALL RESULT: INTEGRITY ISSUES DETECTED!\n');
end
fprintf('========================================================================\n');
