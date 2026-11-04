# Box-folding baseline

Two components remain:

- The MuJoCo box simulation, with the Dexmate robot, table, cameras and low-level motor controller. There is no folding policy, planner, teacher or model-service integration.
- Unmodified [RAX](rax/README.md), pinned to `dexjoco/native` commit `4b577aa2ba0039ad01707ede236be9913274180e`. [RAX_UPSTREAM.json](RAX_UPSTREAM.json) records every upstream file hash. RAX is independent of this custom simulation; connecting them is future work.

Run the simulation from this directory:

```bash
python3 -m venv .venv
.venv/bin/python -m pip install -r requirements.txt
./run.sh build
MUJOCO_GL=egl ./run.sh validate
./run.sh viewer --paused
./run.sh reset
```

The existing `.venv` can be reused. Set `PYTHON_BIN` to use another interpreter. The viewer requires a graphical display; validation runs headlessly with EGL. Press Space to run/pause and R to reset. MuJoCo mouse interaction applies forces to the model.

The blank starts flat. It has 14 finite-thickness rigid panels, 13 passive creases, a free base, gravity, friction and collision contacts. No box actuators, grasp aids, welds or automatic folding are present. Robot motors hold the home pose until supplied new references through `FoldingBoxEnv.step()` or `step_joint_targets()`.

[DESIGN.md](DESIGN.md) describes the physical assumptions. The recorded checks are in [validation/baseline.json](validation/baseline.json). Validation checks numerical stability, mass/inertia, gravity, table support, passive folding under an external torque, timestep sensitivity, reset and rendering. These checks do not establish calibrated cardboard behavior or a working physical lock. Task success remains false pending physical locking validation.

Before cleanup, source and non-ignored evidence were saved in Git commit `c6f5352`. Ignored experiment records were moved to `/home/jennyok/.local/share/box-folding-archive/20261103-230107`, outside this repository. Credentials and the existing virtual environment are excluded from Git.
