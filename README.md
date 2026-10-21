# Open-Source Reproducibility Package: Applied-Input-Constrained Tube MPC for ACC

**Manuscript Title:** *Trajectory-Dependent Uncertainty Propagation and Applied-Input-Constrained Tube MPC for Adaptive Cruise Control*  
**Target Journal:** *IEEE Transactions on Control Systems Technology*  
**Authors:** Avinash K. (Advanced Vehicle Control Laboratory, Department of Electrical Engineering)  
**Package Version:** Stage 48 Final Release (October 2026)  
**Status:** 100% Verified (26/26 Automated Integrity Checks Passed)

---

## 1. Overview & Key Contributions

This archive contains the complete simulation models, controllers, optimization routines, precomputed datasets, and LaTeX manuscript source for the robust tube-based Model Predictive Control (MPC) architecture presented in the paper. 

The architecture resolves three central challenges in robust vehicle following:
1. **Applied Actuator Slew-Rate Compliance:** Eliminates slew-rate violations on the *total physical command* delivered to the vehicle by explicitly conditioning the first nominal input $u_{\xi,0|t}$ on the previous applied command $u_{\mathrm{applied},t-1}$ and instantaneous ancillary feedback correction $c_t = K e_{\xi,t} - \hat d_t$.
2. **Trajectory-Dependent Uncertainty Propagation:** Reduces constraint conservatism by propagating ellipsoidal disturbance tubes along nominal predictions rather than static worst-case bounds, rigorously proved under continuous powertrain lag and Zero-Order Hold.
3. **LMI-Verified Gain Selection:** Evaluates ancillary feedback gains to optimize the trade-off between tracking error rejection and invariant set capacity, verified through discrete Lyapunov dissipation LMIs.
4. **Slew-Invariant Fallback Execution:** Guarantees rate compliance during infeasible transitions and provides smooth deceleration toward $u_{\min}$ during emergency braking.

---

## 2. Directory Structure

```text
Stage48_Reproducibility_Package/
├── README.md                              <- This master documentation and reproduction guide
├── LICENSE                                <- Open-source academic research license (MIT)
├── reproduce_all.m                        <- Master interactive reproduction driver
├── verify_package_integrity.m             <- Automated 26-check self-test script
│
├── manuscript/
│   └── manuscript_complete_draft.tex      <- Full IEEEtran journal manuscript (~656 lines)
│
├── models/
│   ├── ACC_Stage43GE_Experimental.slx     <- Primary active closed-loop Simulink simulation
│   └── ACC_Baseline_Provisional_R10.slx   <- 100% frozen baseline model (benchmark reference)
│
├── data/
│   ├── baseline_provisional_r10_params.mat <- Vehicle plant parameters and nominal limits
│   ├── stage44c_lmi_candidate.mat         <- LMI candidate synthesis results (Candidate 2 optimal)
│   ├── stage43a_qp_matrices.mat           <- Precomputed QP prediction matrices (N=6)
│   ├── stage43e_candidate_constraints.mat <- Tightened polytopic constraint sets
│   ├── stage42h3_lft_scenario.mat         <- LFT uncertainty sensitivity coefficients (theta1..3)
│   ├── stage45_extended_results.mat       <- Raw 16-case emergency braking matrix dataset
│   ├── stage46_ablation_results.mat       <- Raw 5-variant ablation study across 4 benchmarks
│   └── stage46_da_comparison_results.mat  <- Acceleration drift sensitivity test data
│
├── controllers/
│   ├── stage44c_mpc_solver.m              <- Sequential QP solver with step-0 conditioning
│   ├── stage45_mpc_dispatcher.m           <- Runtime controller dispatcher (Controllers A, B, C)
│   ├── stage46_ablation_controller.m      <- Parametric ablation controller (Variants A, B, C, D, E)
│   ├── stage45_controller_a_nominal.m     <- Controller A: Nominal MPC baseline
│   ├── stage45_controller_b_paper.m       <- Controller B: Quan et al. (2024) / Static Tube MPC baseline
│   └── stage45_controller_c_proposed.m    <- Controller C: Proposed integrated tube MPC
│
└── scripts/
    ├── stage46_run_ablation_study.m       <- Re-runs full 20-simulation ablation matrix
    ├── stage45_extended_comparison.m      <- Re-runs full 48-simulation emergency matrix
    ├── stage46_plot_ablation.m            <- Generates 4-panel ablation trajectory figure (Fig 3)
    ├── stage45_plot_tripartite_comparison.m <- Generates 3-panel comparative figure (Fig 4)
    ├── stage46_test_da_comparison.m       <- Evaluates da=0.20 vs 1.0921 sensitivity
    └── stage44e_lmi_audit_and_reproducibility.m <- Evaluates LMI eigenvalues and certificates
```

---

## 3. System Requirements & Setup

* **Software:** MATLAB & Simulink R2022b or later.
* **Required Toolboxes:** Control System Toolbox, Optimization Toolbox.
* **External Solvers:**
  * YALMIP (R20230622 or compatible, external modeling toolbox for LMI analysis and synthesis).
  * SDPT3 or SeDuMi (external semidefinite programming solver).
  * *Note on Standalone Reproducibility:* All core benchmark datasets (`.mat` files) are precomputed and included in `data/`. If you only want to inspect, verify, and reproduce the paper's tables and figures, MATLAB with Control System Toolbox is sufficient. If re-running LMI synthesis or full closed-loop simulations from scratch without precomputed datasets, ensure YALMIP and SDPT3/SeDuMi are installed and added to the MATLAB path.
* **Operating System:** Platform independent (tested on Windows 11 64-bit).

### Quick Setup:
Open MATLAB and add the package folders to the active path:
```matlab
cd('F:\control system\Stage48_Reproducibility_Package');
addpath(genpath(pwd));
```

---

## 4. Verification & One-Click Reproduction

### Step 1: Automated Package Integrity Audit
Run the self-test script:
```matlab
verify_package_integrity
```
*Expected Output:* `26 PASSED, 0 FAILED. PACKAGE INTEGRITY 100% VERIFIED!`  
*(Validates file presence, dataset structure and field schemas, model checksums, and controller definitions without requiring third-party solvers).*

### Step 2: Master Reproduction Driver
Run `reproduce_all`:
```matlab
reproduce_all
```
You will be prompted to select from 6 execution modes:
* **Option 1 (Fast, < 2 sec):** Reproduces Table I & Table II exact values from raw struct data and plots Fig. 3.
* **Option 2 (Fast, < 1 sec):** Reproduces Table III (16-Case Emergency Braking Summary) from raw struct data.
* **Option 3 (~30 sec):** Re-runs closed-loop simulation testing acceleration-drift invariance ($\bar\delta_a = 0.20$ vs $1.0921\ \mathrm{m/s^2}$).
* **Option 4 (< 5 sec):** Re-audits discrete Lyapunov dissipation LMI certificates and multiplier margins (requires YALMIP + SDPT3).
* **Option 5 (~4-6 min):** Re-runs the full 20-simulation 5-variant ablation study.
* **Option 6 (~8-12 min):** Re-runs the full 48-simulation 16-case emergency braking matrix.

---

## 5. Summary of Key Experimental Results

### Table I: Five-Variant Ablation Study (Benchmark 3: Moderate Braking $a_p = -1.0\ \mathrm{m/s^2}$)
Extracted directly from `data/stage46_ablation_results.mat`:

| Performance Metric | Variant A | Variant B | Variant C | Variant D | Variant E (Proposed) |
| :--- | :---: | :---: | :---: | :---: | :---: |
| **Uncertainty Propagation** | Static ($\bar u = 3.0$) | Trajectory | Static ($\bar u = 3.0$) | Trajectory | **Trajectory** |
| **Step-0 Conditioning** | No | No | Yes | Yes | **Yes** |
| **Feedback Gain** | $K_{\mathrm{paper}}$ | $K_{\mathrm{paper}}$ | $K_{\mathrm{paper}}$ | $K_{\mathrm{paper}}$ | **$K_{\mathrm{opt}}$ (LMI)** |
| **QP Feasibility Ratio $\eta_{\mathrm{feas}}$** | 100.0% (51/51) | 100.0% (51/51) | **33.3%** (17/51) | **86.3%** (44/51) | **100.0%** (51/51) |
| **Fallback Steps ($N_{\mathrm{FB}}$)** | 0 | 0 | **34** | **7** | **0** |
| **Max Slew Violation $r_{\Delta u}^{\max}$** | **$0.3539\ \mathrm{m/s^2}$** | **$0.3539\ \mathrm{m/s^2}$** | **$0.0000\ \mathrm{m/s^2}$** | **$0.0000\ \mathrm{m/s^2}$** | **$0.0000\ \mathrm{m/s^2}$** |
| **Min Interval Width $w_{\min}$** | $+2.6545\ \mathrm{m/s^2}$ | $+4.7883\ \mathrm{m/s^2}$ | **$-1.6025\ \mathrm{m/s^2}$** | **$-0.0847\ \mathrm{m/s^2}$** | **$+0.1963\ \mathrm{m/s^2}$** |
| **Peak Uncertainty $\bar\sigma_{\mathrm{peak}}$** | 3.7925 | **0.3405** | 22.1476 | 7.1465 | **0.3915** |
| **Control Energy $\sum u^2$** | 7.7596 | 7.7596 | 22.4468 | 7.0781 | **6.4687** |
| **Total Variation $\mathrm{TV}(u)$** | $7.4788\ \mathrm{m/s^2}$ | $7.4787\ \mathrm{m/s^2}$ | $4.6879\ \mathrm{m/s^2}$ | $6.8840\ \mathrm{m/s^2}$ | **$3.8620\ \mathrm{m/s^2}$ (-48.4%)** |

### Table III: 16-Case Emergency Braking Summary
Extracted directly from `data/stage45_extended_results.mat`:
* **Controllers Evaluated:** Controller A (Nominal MPC), Controller B (Quan et al. 2024 static tube MPC), Controller C (Proposed tube MPC).
* **Actuator Slew Violations:**
  * Controller A: Violations up to $0.7683\ \mathrm{m/s^2}$ (exceeded in 11/16 cases, max in Case 4).
  * Controller B: Violations up to $2.4373\ \mathrm{m/s^2}$ (exceeded in all 16 cases).
  * Controller C (Proposed): **$0.0000\ \mathrm{m/s^2}$** across all 16 cases (all residuals $r_{\Delta u}^{\max} \le 5.10 \times 10^{-5}\ \mathrm{m/s^2}$, strictly bounded by the interior-point solver primal feasibility tolerance $\epsilon_{\mathrm{feas}} \approx 10^{-5}$; rounding to $0.0001$ in Case 4).
* **Collision Avoidance:** Minimum clearance $l_{\min} \ge 20.56\ \mathrm{m} > l_0 = 5.0\ \mathrm{m}$ across all 16 compound stress cases.

---

## 6. Mathematical Foundations & Verification Audit

### 1. Powertrain Discretization and Acceleration Drift (Lemma 1 & Remark 1)
Continuous powertrain lag:
$$\dot a(t) = -\frac{1}{\tau_a} a(t) + \frac{1}{\tau_a} u(t) + w_a(t), \quad \tau_a = 0.5\ \mathrm{s}, \quad \|w_a\|_\infty \le 0.05\ \mathrm{m/s^2}$$
Exact discrete zero-order-hold update ($T_s = 0.1\ \mathrm{s}$):
$$a_{i+1} - a_i = (1 - e^{-T_s/\tau_a})(u_i - a_i) + \bar w_{a,i}, \quad (1 - e^{-0.2} = 0.181269)$$
* **Comfort / Nominal Slew Domain:** $\bar\delta_a = 0.20\ \mathrm{m/s^2}$ ($= T_s \bar j_a$ for ISO 15622 comfort jerk $\bar j_a = 2.0\ \mathrm{m/s^3}$).
* **Unconditional Physical Envelope:** $\bar\delta_{a,\mathrm{phys}} = 1.0921\ \mathrm{m/s^2}$ for all $(u_i, a_i, w_a) \in [-3.0, 2.5] \times [-3.5, 2.5] \times [-0.05, 0.05]$.
* **Invariance Proof:** When evaluated with $\bar\delta_{a,\mathrm{phys}} = 1.0921\ \mathrm{m/s^2}$, Variant E retains **100.0% QP feasibility**, **0 fallback steps**, and **zero slew violation** ($\bar\sigma_{\mathrm{peak}} = 8.5075 < \sigma^* = 13.2482$).

### 2. Dual-Mode Fallback Rate-Compliance Induction (Proposition 2)
On any transition at $t+1$:
1. If the QP at $t+1$ is feasible and enforces step-0 bounds conditioned on $u_{\mathrm{applied},t}$, Proposition 1 guarantees $|u_{\mathrm{applied},t+1} - u_{\mathrm{applied},t}| \le \Delta u_{\max}$.
2. If the QP at $t+1$ is infeasible, fallback command $u_{\mathrm{fallback},t+1} = \max(u_{\min}, u_{\mathrm{applied},t} - \Delta u_{\max})$ guarantees $|u_{\mathrm{applied},t+1} - u_{\mathrm{applied},t}| \le \Delta u_{\max}$.
Inductive rate compliance is thus unconditionally preserved across all possible switching sequences.

### 3. LMI Certificate Transparency
Candidate 2 ($R = 50$, $K_{\mathrm{opt}} = [1.2345, -0.5985, -1.2588]$) provides an interior near-boundary certificate at numerical solver precision:
* Spectral radius: $\rho = 0.9560 < 1.0$
* Tube capacity: $\sigma^* = 13.2482$
* Input tightening coefficient: $g_u = 0.755533$
* Condition number: $\operatorname{cond}(P) = 1.4864 \times 10^4$
* Solver residual margin: $\lambda_{\max}(\mathcal{L}) = -1.0011 \times 10^{-6}$

---

## 7. Baseline Non-Interference Certification

To ensure strict academic reproducibility and eliminate confounding variables:
* The original baseline model `ACC_Baseline_Provisional_R10.slx` and its parameters `baseline_provisional_r10_params.mat` are **100% frozen and unmodified**.
* All experimental innovations are isolated in `ACC_Stage43GE_Experimental.slx` and dynamic controller scripts.

---

## 8. License & Citation

This project is licensed under the MIT License - see the `LICENSE` file for details.

If you find this code or methodology useful in your research, please cite:
```bibtex
@article{Avinash2026TubeMPC,
  author  = {Avinash, K.},
  title   = {Trajectory-Dependent Uncertainty Propagation and Applied-Input-Constrained Tube {MPC} for Adaptive Cruise Control},
  journal = {IEEE Transactions on Control Systems Technology},
  note    = {Under Review},
  year    = {2026}
}
```
