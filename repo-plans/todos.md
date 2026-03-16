Description and intent of the file:
This file is listing down tasks and features that we want before making it into a concrete milestone. There is two  list. The quick todo list is for us to quickly jot down ideas and tasks, while the fleshed out todo list is for us to have a more detailed view of what we want to do and how we want to do it.

---
Quick todo list:
---
Fleshed out todo list:

---

### [P1] motion_planning/multi_robot/level_k — Level-k / Cognitive Hierarchy Reasoning
- **Domain:** `motion_planning/multi_robot/level_k`
- **Reference:** Ren et al., IROS 2021; Camerer et al. cognitive hierarchy model (2004); multiple AV follow-ups 2022–2023.
- **Why:** The entry point for game-theoretic AV planning before tackling ALGAMES/iLQGame. Level-0 agents drive constant velocity. Level-1 agents best-respond to level-0. Level-2 agents best-respond to level-1. No coupled Riccati equations, no augmented Lagrangian — just a nested best-response loop around any single-agent planner. Empirically captures the majority of real human driving behaviour. Implement this first to build the intuition for why Nash equilibrium solvers exist and when they are worth the cost.
- **Key insight:** Most drivers are level-1 or level-2 thinkers. A level-2 autonomous agent outperforms ORCA in interactive scenarios while being only marginally more expensive.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/multi_robot/level_k/`
  - [ ] `Level0Policy` — constant velocity / goal-directed straight-line (configurable)
  - [ ] `LevelKAgent` — templated on depth k; recursively computes best response to level-(k-1) opponent using any `ITrajectoryPlanner` as the inner solver
  - [ ] `CognitiveHierarchyPlanner` — Poisson-distributed mixture over levels (full CHM); soft intent weighting
  - [ ] Tests: 2-agent merge → level-1 yields correctly to level-0; level-2 anticipates level-1 response; benchmark vs ORCA on crossing scenario
  - [ ] `docs/theory.md`: cognitive hierarchy model, best-response definition, convergence to Nash as k→∞, empirical calibration of human level distribution
  - [ ] Wire into `multi_robot/CMakeLists.txt`

### [P1] motion_planning/multi_robot/stackelberg — Stackelberg Game Planner (Asymmetric Merging)
- **Domain:** `motion_planning/multi_robot/stackelberg`
- **Reference:** Multiple groups 2021–2023; Sadigh et al. (Stanford) "Planning for Autonomous Cars that Leverage Effects on Human Actions" (RSS 2016, foundational); follow-up Schmerling et al.; Zhu et al. (T-ITS 2023).
- **Why:** Nash games are symmetric — both agents solve simultaneously. Merging is inherently **asymmetric**: the merging vehicle has the initiative and announces a plan; the highway vehicle reacts. Stackelberg formalises this. The **leader** (merging vehicle) optimizes knowing the follower will best-respond. The **follower** solves a simpler single-agent problem. Solvable in one forward-backward pass — far more tractable than Nash iteration, and more natural for the merging scenario than ALGAMES.
- **Key insight:** Stackelberg equilibrium is computable in polynomial time for LQ dynamics. The leader can *actively influence* the follower's behaviour by choosing a trajectory that makes yielding the follower's rational response.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/multi_robot/stackelberg/`
  - [ ] `StackelbergLeader` — optimizes trajectory assuming follower best-responds; uses iLQR outer loop with follower best-response as an inner constraint
  - [ ] `StackelbergFollower` — single-agent planner that takes leader's committed trajectory as a hard avoidance constraint (can reuse DMPC/ORCA inner solver)
  - [ ] `StackelbergSolver` — coordinates leader/follower solve; role assignment (who is leader) configurable or determined by proximity to merge point
  - [ ] Tests: merge scenario → leader gets priority by influencing follower; verify follower best-response is feasible; compare trajectory quality to Nash (ALGAMES) solution
  - [ ] `docs/theory.md`: Stackelberg vs Nash definition, bilevel optimization formulation, leader influence on follower behaviour, when Stackelberg > Nash (asymmetric scenarios)
  - [ ] Wire into `multi_robot/CMakeLists.txt`

### [P1] motion_planning/multi_robot/contingency_mpc — Contingency / Branch MPC under Intent Uncertainty
- **Domain:** `motion_planning/multi_robot/contingency_mpc`
- **Reference:** Schmerling et al. (Stanford IPRL) — IJRR 2023; McAllister et al. "Robustness to Out-of-Distribution Inputs via Task-Informed Representation Learning" adjacent; "Contingency Model Predictive Control for Autonomous Vehicles" (multiple groups).
- **Why:** iLQGame and ALGAMES assume **known opponent objectives**. In real merging, you don't know if the other car will yield. Contingency MPC solves this by planning **multiple trajectory branches in parallel** — one per plausible opponent intent (e.g., "yields" branch, "doesn't yield" branch). Hard constraints are enforced on *all* branches simultaneously. The agent commits to the branch that remains feasible longest. This is the most directly practical algorithm for the high-speed merging under uncertainty scenario. No open canonical repo but algorithm is fully specified in paper and implementable with OSQP.
- **Key insight:** Instead of estimating intent and planning once, plan for all intents at once and maintain feasibility across all of them. The cost is O(num_branches) × single-agent MPC.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/multi_robot/contingency_mpc/`
  - [ ] `IntentHypothesis` — discrete opponent intent (e.g., `YIELD`, `MAINTAIN_SPEED`, `ACCELERATE`); each carries a prior probability and a predicted opponent trajectory
  - [ ] `ContingencyMPCAgent` — stacks branch subproblems into a single QP; shared initial segment (commitment horizon) + diverging per-branch tail; hard constraints on all branches
  - [ ] `BranchCommitmentPolicy` — monitors branch feasibility online; commits to branch when one intent hypothesis is ruled out by observation
  - [ ] `IntentObserver` — lightweight Bayesian update of intent probabilities from observed opponent state
  - [ ] Tests: 2-agent merge with "yields" and "no yield" hypotheses → shared initial segment is safe under both; intent collapses to correct branch after 3 observations; verify hard constraint never violated on any branch
  - [ ] `docs/theory.md`: branching tree MPC formulation, stacked QP derivation, commitment horizon trade-off, Bayesian intent update, comparison with expected-cost MPC (which fails under bimodal intents)
  - [ ] Wire into `multi_robot/CMakeLists.txt`

### [P1] motion_planning/multi_robot/algames — Augmented Lagrangian Game-Theoretic MPC (High-Speed Merging)
- **Domain:** `motion_planning/multi_robot/algames`
- **Reference:** [Algames.jl (Simon Le Cleac'h)](https://github.com/simon-lc/Algames.jl) — Cleac'h et al., RSS 2020
- **Why:** Best-fit algorithm for high-speed peer-to-peer merging with hard constraints. Models the merging problem as a **Nash game**: each agent minimizes its own trajectory cost (reach goal fast, stay smooth) with hard inter-agent collision-avoidance coupling constraints enforced via augmented Lagrangian. The "who yields" priority decision emerges naturally as a Nash equilibrium — no explicit combinatorial search needed. Addresses the fundamental failure mode of GBPPlanner (soft Gaussian collision factors violated under tight corridor pressure). Reference is Julia; our implementation targets C++ with OSQP.
- **Key concepts:** Augmented Lagrangian for hard constraint enforcement; general-sum game (agents have different costs); KKT conditions per agent; coupled constraint Jacobians; merge corridor as a constrained workspace.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/multi_robot/algames/`
  - [ ] `AlgamesAgent` — per-agent iLQR solver with augmented Lagrangian coupling constraints; receives other agents' current trajectories via P2P comms
  - [ ] `AlgamesCoordinator` — iterates agents' solvers to Nash equilibrium convergence (centralised version for benchmarking); optional decentralised mode where agents share iterates
  - [ ] `CouplingConstraint` — minimum separation hard constraint between agent pairs; evaluates constraint value + Jacobian
  - [ ] `AugmentedLagrangianSolver` — penalty parameter schedule (μ doubling), dual variable updates, convergence check on constraint violation
  - [ ] Merging scenario test: two agents approaching a merge corridor at high speed → converges to one yielding, no collision, trajectories are dynamically feasible
  - [ ] Tests: 2-agent head-on scenario; 4-agent intersection crossing; constraint violation < tolerance at convergence; benchmark iterations-to-convergence vs soft-constraint baseline
  - [ ] `docs/theory.md`: Nash equilibrium formulation, augmented Lagrangian method, KKT conditions, comparison with DMPC (no game theory) and iLQGame (same family, lighter)
  - [ ] Wire into `multi_robot/CMakeLists.txt`; add to M8 or new M8.5 milestone
- **Note:** Start with centralised coordinator for correctness, then add decentralised P2P iterate-sharing variant. Depends on Eigen only (no external QP solver needed for the inner iLQR; OSQP optional for constrained sub-problems).

### [P1] motion_planning/multi_robot/ilqgame — Iterative LQR for General-Sum Multi-Agent Games
- **Domain:** `motion_planning/multi_robot/ilqgame`
- **Reference:** [ilqgames (Fridovich-Keil et al.)](https://github.com/HJReachability/ilqgames) — ICRA 2020; C++, open source
- **Why:** Lighter-weight game-theoretic planner in the same family as ALGAMES. Extends the iLQR algorithm (already understood from our control modules) to multi-player general-sum games by solving coupled Riccati equations per agent. Converges to local Nash equilibria. Faster per iteration than ALGAMES (no augmented Lagrangian penalty loop), but less robust convergence in highly constrained scenarios. C++ open-source reference with merging/intersection demos makes it an excellent learning companion to ALGAMES — implement both to understand the trade-offs.
- **Key difference from ALGAMES:** Uses iLQR's backward pass to compute per-agent linear feedback policies simultaneously; hard constraints via projection rather than augmented Lagrangian → faster but softer safety.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/multi_robot/ilqgame/`
  - [ ] `ILQGameSolver` — coupled backward pass solving per-agent Riccati equations; forward pass with line search; shared `GameState` containing all agents' states
  - [ ] `PlayerCost` — per-agent quadratic cost (goal tracking + control effort + proximity penalty)
  - [ ] `GameDynamics` — stacked single-agent dynamics; computes coupled Jacobians
  - [ ] Constraint handling: soft quadratic barrier (base), optional hard projection step for collision constraints
  - [ ] Tests: 2-agent merging → converges to Nash; 4-agent roundabout → no deadlock; benchmark convergence speed vs ALGAMES
  - [ ] `docs/theory.md`: general-sum game formulation, coupled Riccati backward pass derivation, comparison with ALGAMES, when to prefer iLQGame vs ALGAMES
  - [ ] Wire into `multi_robot/CMakeLists.txt`

### [P2] motion_planning/multi_robot/opponent_modeling — Online Bayesian Opponent Objective Estimation
- **Domain:** `motion_planning/multi_robot/opponent_modeling`
- **Reference:** Tian et al. — "Learning to Play Trajectory Games Against Opponents with Unknown Objectives", RA-L 2023. Companion to [ilqgames](https://github.com/HJReachability/ilqgames).
- **Why:** iLQGame and ALGAMES require knowing the opponent's cost function. This module estimates it online from observed behaviour and feeds the estimate back into any game solver. Each timestep: observe opponent state → Bayesian update over a discrete set of cost hypotheses (aggressive, passive, cooperative) → pass MAP or weighted-mixture cost to iLQGame/ALGAMES inner solver. Makes any game-theoretic planner applicable to real AV scenarios where intent is unknown. Clean separation: this module is a pure estimation layer, not a planner itself.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/multi_robot/opponent_modeling/`
  - [ ] `CostHypothesis` — parameterised opponent cost function (goal weight, collision aversion, speed preference); discrete library of N hypothesis types
  - [ ] `BayesianOpponentEstimator` — maintains probability distribution over `CostHypothesis` set; likelihood = how well hypothesis predicts last K observed states; update via Bayes rule
  - [ ] `OpponentModelInterface` — outputs MAP hypothesis or full posterior for use by any game solver (`ILQGameSolver`, `AlgamesSolver`)
  - [ ] Tests: aggressive opponent observed for 5 steps → posterior concentrates on aggressive hypothesis; feed MAP into iLQGame → solution differs from uniform prior; verify posterior collapses correctly across all hypothesis types
  - [ ] `docs/theory.md`: Bayesian inverse optimal control formulation, likelihood model derivation, how posterior uncertainty propagates through game solver, connection to level-k (level-k uses structural assumptions; this uses observed data)
  - [ ] Wire into `multi_robot/CMakeLists.txt`

### [P2] motion_planning/multi_robot/cbf_game — CBF Safety Filter for Game-Theoretic Planners
- **Domain:** `motion_planning/multi_robot/cbf_game`
- **Reference:** Zheng et al., "Safe iLQGame", ICRA 2024; Herbert et al. (Stanford); Zeng et al. CBF+MPC (RA-L 2021).
- **Why:** iLQGame and ALGAMES produce Nash trajectories but have no hard safety certificate — the game solver may drift if opponent deviates from the Nash assumption. This module is a thin safety filter that wraps any game-theoretic trajectory plan: at each control timestep it takes the game planner's desired acceleration/velocity and projects it onto the safe set defined by a Control Barrier Function. Hard collision invariance is guaranteed even if the game solver's assumptions break. Reuses our existing `control/cbf/` module — implementation cost is low. This is the production-safe version of any game planner.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/multi_robot/cbf_game/`
  - [ ] `MultiAgentCBF` — pairwise CBF between all agent pairs; h(x) = ‖pᵢ - pⱼ‖² - (rᵢ + rⱼ + margin)²; vectorised over N(N-1)/2 pairs
  - [ ] `CbfGameFilter` — takes nominal control from any game planner + current multi-agent state; solves a small QP (OSQP) to find minimal-deviation control satisfying all pairwise CBF constraints
  - [ ] Integration adapters: `ILQGameCbfAgent`, `AlgamesCbfAgent` — composable wrappers
  - [ ] Tests: iLQGame nominal plan + CBF filter → hard separation invariant holds when opponent deviates from Nash; verify QP solve < 0.5ms for N=6 agents
  - [ ] `docs/theory.md`: CBF invariance proof for multi-agent case, QP formulation, when game planner + CBF is sufficient vs when CBF alone suffices (ORCA), performance overhead analysis
  - [ ] Wire into `multi_robot/CMakeLists.txt`
- **Note:** Depends on `control/cbf/` — must be implemented first.

### [P2] motion_planning/multi_robot/consensus_admm — Consensus ADMM Multi-Agent Trajectory Optimization
- **Domain:** `motion_planning/multi_robot/consensus_admm`
- **Reference:** No single canonical repo — algorithm based on Boyd et al. "Distributed Optimization and Statistical Learning via ADMM" (2011) applied to multi-agent trajectory optimization.
- **Why:** This is conceptually "GBPPlanner with hard constraints." GBPPlanner uses Gaussian Belief Propagation over a factor graph — each inter-agent collision factor is a Gaussian (soft). ADMM uses the same factor-graph decomposition but enforces inter-agent constraints exactly via dual variable (Lagrange multiplier) updates. Each agent solves a local trajectory optimization subproblem; a consensus step drives agreement on shared collision-avoidance constraints. At convergence, hard constraints are satisfied by construction. Pedagogically bridges the gap between GBP (which the user already knows) and the game-theoretic approaches.
- **Key insight:** Replace GBP's Gaussian message passing with ADMM's dual variable updates. Same local subproblem structure, hard constraint guarantee instead of soft.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/multi_robot/consensus_admm/`
  - [ ] `AdmmAgent` — solves local trajectory QP (OSQP) with augmented Lagrangian terms for coupling constraints; broadcasts primal trajectory and receives neighbors' trajectories
  - [ ] `ConsensusLayer` — maintains dual variables (λ) and penalty parameter (ρ); updates λ after each primal solve; broadcasts consensus targets
  - [ ] `CouplingConstraint` — minimum separation constraint linearised around current trajectory; forms the A matrix in the ADMM split
  - [ ] Warm-starting: previous solution as initial iterate for real-time replanning
  - [ ] Tests: 2-agent merging → hard separation constraint satisfied at convergence; compare constraint violation vs GBP baseline; test with increasing ρ schedule
  - [ ] `docs/theory.md`: ADMM split for trajectory optimization, dual variable update derivation, connection to GBP (message passing = dual updates), convergence rate vs penalty schedule
  - [ ] Wire into `multi_robot/CMakeLists.txt`

### [P2] motion_planning/multi_robot/bvc — Buffered Voronoi Cells
- **Domain:** `motion_planning/multi_robot/bvc`
- **Reference:** Zhu & Alonso-Mora — "Chance-Constrained Collision Avoidance for MAVs in Dynamic Environments", T-RO 2019. See also [bvc implementations](https://github.com/Mit-agility/bvc) and related Alonso-Mora group repos.
- **Why:** Simplest hard-constraint decentralized planner, and surprisingly effective for merging. Each agent computes its Voronoi cell w.r.t. neighbors, shrinks it by a safety buffer (= robot radius + margin), then solves a local QP to stay within its buffered cell. Hard constraint by construction — agents cannot enter each other's buffered cells. The Voronoi partition naturally encodes priority in merging (the agent closer to the merge point gets the larger cell). O(N log N) per agent. Real-time. Excellent reactive safety layer that can run under ALGAMES or iLQGame.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/multi_robot/bvc/`
  - [ ] `VoronoiPartition` — computes 2D/3D Voronoi tessellation from agent positions (Fortune's algorithm or incremental; Eigen-only)
  - [ ] `BufferedVoronoiCell` — shrinks Voronoi cell by safety buffer; expresses cell as set of half-plane constraints
  - [ ] `BvcAgent` — solves local QP (OSQP) to find velocity/waypoint within buffered cell closest to goal direction
  - [ ] Tests: 2-agent merge → each stays within its cell, no collision; 10-agent random → no pairwise cell overlap; benchmark QP solve time < 1ms
  - [ ] `docs/theory.md`: Voronoi tessellation, buffer sizing derivation, half-plane QP formulation, why this handles merging priority implicitly
  - [ ] Wire into `multi_robot/CMakeLists.txt`
- **Note:** Natural pairing with ALGAMES: BVC as the hard reactive safety filter, ALGAMES for trajectory-level planning.

### [P2] motion_planning/multi_robot/rmader — Robust MADER: Decentralized Multi-Agent Trajectory Planning Robust to Communication Delay
- **Domain:** `motion_planning/multi_robot/rmader`
- **Reference:** [RMADER (MIT ACL)](https://github.com/mit-acl/rmader) — Kondo et al., IEEE RA-L + ICRA 2023, arXiv:2303.06222
- **Why:** MADER is already implemented (`multi_robot/mader/`) but assumes reliable, low-latency comms. RMADER is the direct successor that adds a **delay compensation step**: agents hold their candidate trajectory for a delay buffer period before committing, ensuring no neighbor's delayed broadcast can arrive after commitment. Validated at 300ms network delay with 10 agents + dynamic obstacles on hardware. This is the production-ready version of MADER for real-world deployments.
- **Key difference from MADER:** Adds a `DelayBuffer` holding window before the check-commit step; the rest of the pipeline (convex decomposition, MINVO intersection check, Bézier polynomial trajectory) is shared.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/multi_robot/rmader/`
  - [ ] `RmaderAgent : MaderAgent` (or composition) — inherits/wraps MADER pipeline, adds delay buffer and hold-before-commit logic
  - [ ] `RmaderConfig` — extends `MaderConfig` with `delay_budget_ms`, `hold_time_s`
  - [ ] `DelayBuffer` — time-stamped trajectory queue; flushes stale entries; triggers recheck on late arrivals
  - [ ] Shared types with `mader/` via `common/` (no cross-module linking — types passed by value)
  - [ ] Tests: single agent under simulated delay → commits only after hold window; two agents with artificial 200ms delay → no collision in committed segments
  - [ ] `docs/theory.md`: delay compensation proof sketch, comparison with MADER safety guarantee, when RMADER vs MADER is appropriate
  - [ ] Wire into `multi_robot/CMakeLists.txt`; add to M8 or M12 milestone
- **Note:** Reference implementation uses Gurobi; our version should use OSQP (already in `deps.cmake`) — same trade-off as MADER.

### [P3] motion_planning/trajectory_planning/hermite_spline — Hermite Spline-based Efficient Trajectory Planning (MIGHTY)
- **Domain:** `motion_planning/trajectory_planning/hermite_spline`
- **Reference:** [MIGHTY (MIT ACL)](https://github.com/mit-acl/mighty) — Kondo et al., arXiv:2511.10822, submitted to IEEE RA-L
- **Why:** MIGHTY introduces Hermite spline parameterization as a computationally efficient alternative to minimum-snap polynomial trajectories. Hermite splines give direct control over endpoint positions and velocities, enabling fast replanning in cluttered and dynamic environments. It is distinct from our existing `polynomial` (min-snap) and `spline_fitting` modules. Proven on multi-agent UAV hardware in dense forests with dynamic obstacles.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/trajectory_planning/hermite_spline/`
  - [ ] `HermiteSplineTrajectory` — cubic Hermite spline segments parameterized by endpoint positions + velocities; supports N-segment chains
  - [ ] `HermiteTrajectoryPlanner` — generates collision-free trajectories by optimizing knot velocities; convex-hull safety constraint via `DecompUtil`-style corridor (Eigen-only approximation)
  - [ ] Dynamic obstacle avoidance: re-plan over a receding horizon when obstacle proximity triggers replanning
  - [ ] Multi-agent support: inter-agent collision constraints as soft penalty terms
  - [ ] `ITrajectoryPlanner` conformance (same interface as `polynomial`, `teb`)
  - [ ] Tests: single-segment endpoint boundary conditions; multi-segment continuity; replanning triggered by obstacle; convergence to goal within horizon
  - [ ] `docs/theory.md`: Hermite basis functions, knot velocity optimization, comparison with min-snap, receding-horizon replanning loop
  - [ ] Add to appropriate milestone (M16 Planning Upgrades II or new M16.5)


### [P5] motion_planning/multi_robot/gbp_planner — Gaussian Belief Propagation Multi-Agent Planner
- **Domain:** `motion_planning/multi_robot/gbp_planner`
- **Reference:** [gbpplanner (Patwardhan et al.)](https://github.com/aalpatya/gbpplanner) — Patwardhan, Bhatt & Stachniss, ICRA 2023
- **Why (low priority):** GBP is a beautiful factor-graph algorithm for joint trajectory optimization — each agent's trajectory and inter-agent collision avoidance are modelled as factors in a probabilistic graph; belief propagation finds a consensus solution. Pedagogically interesting as an alternative to optimization-based approaches. However, collision factors are Gaussian (soft constraints) — the planner degrades in tight high-speed scenarios where constraints cannot be softened. **Add primarily for comparison and learning; ALGAMES/iLQGame are the correct choice for hard-constraint merging.** Open source C++, clean implementation.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/multi_robot/gbp_planner/`
  - [ ] `FactorGraph` — variable nodes (waypoints) + factor nodes (dynamics, obstacle, inter-agent); sparse message-passing structure
  - [ ] `DynamicsFactor` — enforces kinematic continuity between waypoints
  - [ ] `InterAgentFactor` — Gaussian collision cost between pairs of agents' waypoint variables
  - [ ] `GbpSolver` — iterative belief propagation (fixed iterations or convergence check); produces mean trajectory per agent
  - [ ] Tests: 2-agent low-speed crossing → avoidance emerges; document constraint violation rate under tight merge corridor (expected to be non-zero — this is a known limitation)
  - [ ] `docs/theory.md`: factor graph formulation, Gaussian belief propagation message update equations, why soft constraints fail under hard geometric pressure, comparison with ALGAMES
  - [ ] Wire into `multi_robot/CMakeLists.txt`

### [P3] motion_planning/multi_robot/potential_game — Potential Game Formulation for Multi-Agent Driving
- **Domain:** `motion_planning/multi_robot/potential_game`
- **Reference:** Mehr et al. (Stanford) — "Maximal Liveness of Multi-Agent Systems via Multi-Objective Optimization" + "Autonomous Vehicles on the Road", CDC 2021; Monderer & Shapley (1996) foundational.
- **Why:** Nash equilibrium in a general-sum game requires expensive coupled iteration (ALGAMES/iLQGame). If the multi-agent problem can be cast as a **potential game** — where a single scalar potential function Φ exists such that any unilateral cost change equals the change in Φ — then Nash equilibrium reduces to minimizing Φ. Gradient descent on a single function. Orders of magnitude faster than coupled Riccati equations. Applicable when agents share infrastructure costs (merge timing gaps, road congestion, fuel efficiency). Teaches a fundamentally different path to Nash: instead of iterating coupled best-responses, find the problem structure that makes Nash cheap.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/multi_robot/potential_game/`
  - [ ] `PotentialFunction` — defines Φ over joint trajectory space; parameterised by per-agent goal costs + shared coupling terms (separation penalty, merge timing cost)
  - [ ] `PotentialGameSolver` — gradient descent / L-BFGS on Φ over joint trajectory; step size via Armijo line search; Eigen-only
  - [ ] `PotentialGameVerifier` — checks whether a given multi-agent cost structure admits a potential function (finite-improvement property test on small examples)
  - [ ] Tests: 2-agent merge → potential game Nash matches ALGAMES solution within tolerance at 10× speed; verify finite-improvement property holds; document failure case where potential structure breaks down
  - [ ] `docs/theory.md`: potential game definition, existence conditions, relationship to Nash equilibrium, congestion game examples, when the structure holds for AV (and when it doesn't)
  - [ ] Wire into `multi_robot/CMakeLists.txt`

### [P3] motion_planning/multi_robot/interaction_aware — Interaction-Aware Prediction + Game-Theoretic Planning
- **Domain:** `motion_planning/multi_robot/interaction_aware`
- **Reference:** Multiple groups IROS/ICRA 2022–2024; Ivanovic & Pavone (Stanford) "TRAJECTRON++"; Salzmann et al.; Sun et al. "M2I" (CVPR 2022).
- **Why:** Separates the prediction problem (what will opponents do?) from the planning problem (what should I do given that?). Uses a learned or parametric **conditional prediction model** — predicts the opponent's trajectory distribution *conditioned on my planned trajectory*. Feeds this interaction-aware prediction into any game solver or MPC. The key upgrade over naive prediction-then-plan: the prediction model knows that opponents will react to you, so it avoids the "frozen robot" problem where the ego plan is over-cautious because predictions don't account for interaction. Bridges deep learning prediction literature into our C++ game theory stack via a clean `IInteractionPredictor` interface.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/multi_robot/interaction_aware/`
  - [ ] `IInteractionPredictor` — interface: given ego trajectory + opponent history → returns distribution over opponent future trajectories
  - [ ] `ConstantVelocityPredictor` — baseline: ignores ego plan (frozen robot); useful for benchmarking
  - [ ] `ReactivePredictor` — parametric: opponent maintains time gap to ego; models courtesy/aggressiveness via a single parameter α
  - [ ] `InteractionAwareMPC` — wraps any `DmpcAgent` or `AlgamesAgent`; uses `IInteractionPredictor` to generate opponent trajectory distribution; optimizes over expected cost weighted by prediction
  - [ ] Tests: frozen robot vs reactive predictor in merge → reactive predictor allows ego to merge more assertively; verify ego trajectory is less conservative with interaction-aware prediction
  - [ ] `docs/theory.md`: frozen robot problem, conditional prediction vs marginal prediction, interaction-awareness as an implicit game (connection to iLQGame), plug-in interface for learned predictors
  - [ ] Wire into `multi_robot/CMakeLists.txt`

### [P4] motion_planning/multi_robot/differential_game_racing — Time-Optimal Differential Games for Autonomous Racing
- **Domain:** `motion_planning/multi_robot/differential_game_racing`
- **Reference:** Liniger & Lygeros — "A Noncooperative Game Approach to Autonomous Racing", T-AC 2020; Spica et al. IROS 2020; follow-ups 2022–2024. Related: [MPCC racing controller](https://github.com/alexliniger/MPCC).
- **Why:** Zero-sum differential game applied to head-to-head racing and overtaking. Your gain (lap time, position) is exactly the opponent's loss. The optimal overtaking and blocking strategies are computable from the value function of this zero-sum game. For 2-player LQ approximations, Hamilton-Jacobi reachability gives the exact solution. Pedagogically distinct from everything else in the stack — introduces zero-sum games, saddle-point equilibria, and the connection between game theory and reachability analysis. The `MPCC` open-source racing controller (Liniger, ETH) provides a strong C++ reference.
- **Scope:**
  - [ ] Scaffold `workspace/robotics/motion_planning/multi_robot/differential_game_racing/`
  - [ ] `ZeroSumRacingGame` — 2-player, joint state = [ego_state, opponent_state]; cost = ego lap progress - opponent lap progress; saddle-point equilibrium via minimax
  - [ ] `MinimaxiLQR` — iLQR extended to minimax: ego maximizes, opponent minimizes (or vice versa); alternating gradient steps on coupled Riccati
  - [ ] `OvertakingStrategy` — high-level policy: when to attempt overtake (value function threshold), which side to attack (left/right), blocking detection
  - [ ] `RacingLineReference` — single-agent time-optimal racing line (minimum curvature / MPCC-style) as the nominal trajectory around which the game is linearized
  - [ ] Tests: 2-car approach to chicane → optimal overtake emerges; blocking strategy prevents naive overtake; benchmark saddle-point convergence; compare to greedy (ignore opponent) baseline lap time
  - [ ] `docs/theory.md`: zero-sum vs general-sum, saddle-point equilibrium, minimax iLQR derivation, connection to Hamilton-Jacobi reachability (value function = safety margin), when zero-sum assumption is appropriate (racing) vs inappropriate (merging)
  - [ ] Wire into `multi_robot/CMakeLists.txt`
- **Note:** Requires `trajectory_planning/time_optimal/` as dependency for the racing line reference. High implementation complexity — tackle after ALGAMES + iLQGame are solid.
