# Multi-Agent Persistent Monitoring on Networks — Web Simulator

Browser-based (JavaScript / p5.js) simulator for the multi-agent persistent monitoring problem on
graphs: a team of agents moves between target nodes whose uncertainty grows while unobserved and
decays while an agent is present, and the goal is to minimize the time-averaged total uncertainty.

Live version: https://www.bu.edu/codes/simulations/shiran27/PersistentMonitoring/
(this repository mirrors that page; last synced October 2026).

Developed by Shirantha Welikala, supervised by Prof. Christos G. Cassandras (Boston University, CODES Lab).

## Running

No build step. Clone the repository and open `index.html` in a browser (Chrome or Firefox). Some
browsers restrict `file://` access to scripts, in which case serve the folder locally, e.g.
`python3 -m http.server 8000` and open http://localhost:8000/.

Quick-start sequences and a labelled screenshot of the interface are in `PMGWebSimReport.pdf`.

## What it does

- **Problem configuration**: place targets and agents on the canvas (or load a preset: 1–3 agents on
  3–9 targets, maze, room, random), set per-target uncertainty rates, agent sensing rates, and the
  network edges.
- **Control methods** (`Control Method` dropdown): TCP method (threshold-based, RHC disabled);
  one- and two-step greedy RHC; Event-Driven RHC in several variants (fixed horizon, adaptive
  horizon, alpha-weighted, two-step, two-step alpha–beta); ED-ORHC for second-order agents (SO*, SO#)
  and first-order agents (FO1–FO3) with actuator limits (v_max, u_max); learned classifiers;
  random and random-RL baselines; Event-Driven RHC with (approximate) reinforcement learning;
  and a brute-force exhaustive search (`trace.js`) for small instances.
- **Threshold-based control** with distributed gradient optimization, boosting functions, step-size
  selection, spectral-clustering initialization, and greedy initialization (see the dropdowns under
  Problem Configuration).
- **Plots**: live objective J(t), and a separate window (`plot.html`) with state trajectories.

## Code layout

| File | Role |
|---|---|
| `index.html`, `style.css` | UI (Bootstrap) |
| `sketch.js` | p5.js draw loop; dispatches the selected control method each step |
| `simulation.js` | simulation state, UI callbacks, parameter handling |
| `agent.js` | agent class; RHC / ED-RHC / ED-ORHC solvers, RL updates |
| `target.js`, `path.js`, `cycle.js`, `point2.js` | targets, edges/paths, cycles, geometry |
| `trace.js` | brute-force exploration traces |
| `rationalObj.js`, `mathTools.js` | symbolic/rational-function helpers, numerics |
| `plot.js`, `plot.html` | Plotly plots |
| `libraries/` | p5.js, Plotly, math.js, numeric.js, nerdamer 1.1.4 (`libraries/nerd/`), jQuery, Bootstrap |

Note: `index.html` still references `event.js`, `obstacle.js`, `particle.js` and
`libraries/algebra-0.2.5.min.js`, which do not exist (also absent on the live page); the browser
ignores them.

## Related

- S. Welikala and C. G. Cassandras, "Event-Driven Receding Horizon Control for Distributed Persistent
  Monitoring in Network Systems," Automatica, 2021 (arXiv:2003.11713).
- S. Welikala and C. G. Cassandras, "Event-Driven Receding Horizon Control of Energy-Aware Dynamic
  Agents for Distributed Persistent Monitoring," IEEE CDC 2021 (arXiv:2102.12963).
- MATLAB successor simulator (Stevens Institute of Technology, ADIISLab):
  https://github.com/shiran27/MultiAgentSimulator
