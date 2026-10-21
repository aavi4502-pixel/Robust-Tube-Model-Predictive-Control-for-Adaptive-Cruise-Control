# One-Page Defense Cheat Sheet: Robust Tube MPC for ACC

**Target:** Thesis Defense / Professor Q&A / Review Committee  
**Paper:** *Trajectory-Dependent Uncertainty Propagation and Applied-Input-Constrained Tube MPC for Adaptive Cruise Control* (IEEE TCST Under Review)  
**Author:** Avinash K. (AVCL, Department of Electrical Engineering)

---

## 1. Core Equation-to-Code Traceability Matrix

| # | Theoretical Concept & Base Equation | Exact Code Location | One-Line Engineering Justification |
| :-: | :--- | :--- | :--- |
| **1** | **Continuous Vehicle Plant State-Space:**<br>$\dot x = A_c x + B_c u + B_{p,c} a_p + B_{w,c} w_a$<br>$A_c = \begin{bmatrix} 0 & 1 & -\tau_h \\ 0 & 0 & -1 \\ 0 & c/\tau_a & -1/\tau_a \end{bmatrix}$ | `baseline_provisional_r10_params.mat`<br>`init_workspace.m:15–25` | Models 3rd-order longitudinal spacing kinematics with headway $\tau_h = 1.5\text{ s}$, engine lag $\tau_a = 0.5\text{ s}$, and linearized aerodynamic drag $c = C_0 v_0 = 2.0\text{ s}^{-1}$. |
| **2** | **Exact ZOH Acceleration Update:**<br>$a_{i+1} - a_i = (1 - e^{-T_s/\tau_a})(u_i - a_i) + \bar w_{a,i}$<br>$(1 - e^{-0.1/0.5} = 0.181269)$ | `stage46_test_da_comparison.m:40–55`<br>`manuscript:188` | Analytical step response of powertrain lag over $T_s = 0.1\text{ s}$; proves acceleration increments are bounded by $\bar\delta_a = 0.20\text{ m/s}^2$ (comfort) and $\bar\delta_{a,\mathrm{phys}} = 1.0921\text{ m/s}^2$ (unconditional). |
| **3** | **Nominal State & Input Horizon ($N=6$):**<br>$\xi_{k+1\|t} = \Phi \xi_{k\|t} + \Gamma u_{\xi,k\|t} + \Gamma_a a_{p,k\|t}$ | `stage43a_qp_matrices.mat`<br>`stage44c_mpc_solver.m:189–193` | 600 ms lookahead horizon ($N=6, T_s=0.1\text{ s}$); balances preview horizon against real-time solve budget (median 103 ms). |
| **4** | **Trajectory Perturbation Bound (Lemma 1):**<br>$\bar z_{\xi,k\|t} = \theta_1 \|\Delta v\| + (k T_s \theta_1 + \theta_2) \|a\| + \dots + \theta_3 \|u\|$ | `stage44c_mpc_solver.m:194–201` | Replaces overly conservative static bounds by propagating disturbance ellipses strictly along the planned nominal trajectory. |
| **5** | **Ellipsoidal Uncertainty Contraction:**<br>$\bar\sigma_{k+1\|t} = \lambda \bar\sigma_{k\|t} + \frac{\alpha_k^2}{1-\lambda}, \quad \lambda = \rho^2$ | `stage44c_mpc_solver.m:211–225`<br>`stage46_ablation_controller.m:54, 61` | Proves recursive invariant tube contraction under Lyapunov decay rate $\rho = 0.9560$ ($\rho^2 = 0.9139$). |
| **6** | **4-Way Step-0 Bounds (Proposition 1):**<br>$u_{\xi 0,\mathrm{LB}} = \max(u_{\min} - c_t, \, u_{\mathrm{app},t-1} - \Delta u_{\max} - c_t, \, u_{\xi,\mathrm{prev}} - \Delta u_{\max}, \, u_{\min} + \hat d_t)$<br>$u_{\xi 0,\mathrm{UB}} = \min(u_{\max} - c_t, \, u_{\mathrm{app},t-1} + \Delta u_{\max} - c_t, \, u_{\xi,\mathrm{prev}} + \Delta u_{\max}, \, u_{\max} + \hat d_t)$ | `stage44c_mpc_solver.m:177–185` | **The Core Invention:** Directly conditions the QP decision variable so that the total applied command $u_{\mathrm{app}} = u_{\xi,0} + c_t$ strictly satisfies the physical actuator slew limit $\Delta u_{\max} = 0.25\text{ m/s}^2$. |
| **7** | **Interval Collapse Detector:**<br>$w_0 = u_{\xi 0,\mathrm{UB}} - u_{\xi 0,\mathrm{LB}} \ge 0$ | `stage44c_mpc_solver.m:187, 204` | If $w_0 < 0$ (lower bound > upper bound), the set is empty; instantly flags infeasibility and transitions safely to fallback. |
| **8** | **LMI Dissipation Certificate (Theorem 1):**<br>$\begin{bmatrix} \Phi_e^\top P \Phi_e - \rho^2 P & \Phi_e^\top P \Gamma_e & C_e^\top \\ \star & \Gamma_e^\top P \Gamma_e - \gamma^2 I & 0 \\ \star & \star & -I \end{bmatrix} \le 0$ | `stage44e_lmi_audit_and_reproducibility.m:160–210`<br>`data/stage44c_lmi_candidate.mat` | Rigorous Semidefinite Program (SDPT3) certificate proving closed-loop stability, multiplier margin $> 0$, and decay $\rho = 0.9560$. |
| **9** | **Ancillary Feedback Synthesis ($K_{\mathrm{opt}}$):**<br>$[K_{\mathrm{opt}}, ~] = \operatorname{dlqr}(\Phi, \Gamma, \operatorname{diag}(100,50,10), 50)$<br>$K_{\mathrm{opt}} = [1.2345, -0.5985, -1.2588]$ | `stage44e_lmi_audit_and_reproducibility.m:142–144` | Synthesized via discrete LQR ($R=50$) to penalize ancillary control effort and certified by Theorem 1 LMI. |
| **10** | **Input Tightening Footprint & Capacity:**<br>$g_u = \sqrt{K (P^{-1})_{11} K^\top} = 0.755533$<br>$\sigma^* = \left(\frac{u_{\max}-u_{\min}}{2 g_u}\right)^2 = 13.2482$ | `stage44e_lmi_audit_and_reproducibility.m:145–147`<br>`data/stage44c_lmi_candidate.mat` | Quantifies the maximum allowable uncertainty before interval collapse; $K_{\mathrm{opt}}$ expands capacity by **+21.66%** over baseline ($\sigma^*=10.8893$). |
| **11** | **Slew-Invariant Fallback (Proposition 2):**<br>$u_{\mathrm{fallback}}(t) = \max(u_{\min}, \, u_{\mathrm{app}}(t-1) - \Delta u_{\max})$<br>$u_{\xi,\mathrm{cmd}} = u_{\mathrm{fallback}}(t) - c_t$ | `stage44c_mpc_solver.m:295–310` | Injects monotonic deceleration through the summing junction, ensuring $|u(t) - u(t-1)| \le \Delta u_{\max}$ across all switching sequences. |
| **12** | **Total Applied Command to ECU:**<br>$u_{\mathrm{applied}}(t) = \operatorname{sat}_{[u_{\min}, u_{\max}]}(u_{\xi,\mathrm{cmd}} + K e_{\xi} - \hat d)$ | `ACC_Stage43GE_Experimental.slx`<br>`stage45_mpc_dispatcher.m:20–30` | Real-time physical signal sent to vehicle driveline actuator. |

---

## 2. Instant Q&A Cheat Sheet (For Committee & Professor)

### Q1: "Why does the standard tube MPC violate actuator limits if MPC is constrained?"
> **Answer:** "Standard tube MPC only constrains the *nominal* input $u_\xi$ inside the solver. The actual command sent to the car adds ancillary feedback $u_{\mathrm{app}} = u_\xi + K e - \hat d$ *outside* the solver. When a disturbance or sudden braking occurs, $K e$ spikes, violating the actuator slew rate limit (+141.6% in Core-3, and up to +974.9% in emergency stops)."

### Q2: "Why couldn't you just add rate constraints to the nominal model?"
> **Answer:** "If you naively add rate limits alongside static worst-case disturbance bounds (Variant C), the constraint set undergoes **input-interval collapse** ($w_{\min} = -1.60\text{ m/s}^2$). The lower bound becomes larger than the upper bound, causing the QP solver to fail in 34 out of 51 steps (only 33.3% feasible). Our paper solves this by combining trajectory bounds with LMI gain co-design."

### Q3: "What is the difference between Variant D (86.3%) and Variant E (100.0%)?"
> **Answer:** "Both use trajectory bounds and step-0 rate constraints. The only difference is the feedback gain: Variant D uses the literature gain from Quan et al. ($K_1 = 2.4855$), which is too aggressive and collapses the interval in 7 steps ($w_{\min} = -0.0847\text{ m/s}^2$). Variant E uses our co-designed gain $K_{\mathrm{opt}}$ ($R=50$), which expands the collapse margin by +21.66% and keeps $w_{\min} = +0.1963\text{ m/s}^2 > 0$ at all times, restoring 100% feasibility."

### Q4: "Why do you claim 0.0000 m/s² slew violation when Case 4 shows 0.0001 m/s²?"
> **Answer:** "The residual in Case 4 is $5.10 \times 10^{-5}\text{ m/s}^2$, which rounds to $0.0001$ at 4 decimals. This is strictly the primal feasibility convergence tolerance ($\epsilon_{\mathrm{feas}} \approx 10^{-5}$) of the interior-point numerical solver, representing zero structural violation. In contrast, baseline controllers exhibit massive structural violations up to $2.4373\text{ m/s}^2$."
