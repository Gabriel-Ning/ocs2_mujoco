import mujoco
import mujoco.viewer
import numpy as np
import time
import pathlib
import scipy.spatial.transform
import qpsolvers
import mink


def main():
    # Paths
    current_dir = pathlib.Path(__file__).parent.absolute()
    model_dir = current_dir.parent / "models"
    xml_path = model_dir / "simulation" / "scene.xml"

    # Load MuJoCo
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    # Setup Mink
    configuration = mink.Configuration(model)

    # Tasks
    # We want to track the "ee_site" (Green dot in simulation)
    # The target is the "target" Mocap body.
    ee_task = mink.FrameTask(
        frame_name="ee_site",
        frame_type="site",
        position_cost=10.0,
        orientation_cost=10.0,
        lm_damping=1e-2,  # Damping
    )

    # Posture task (Nullspace bias)
    posture_task = mink.PostureTask(model=model, cost=1e-3)
    # Set nominal posture
    q_nom = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0])
    # Posture task expects full init q? Or just joints?
    q_nom_full = np.zeros(model.nq)
    q_nom_full[:7] = q_nom
    posture_task.set_target(q_nom_full)

    tasks = [ee_task, posture_task]

    # Initialization
    initial_joints = [0.0, 0.171, 0.114, -1.570, 0.050, 1.570, 0.469]
    data.qpos[:7] = initial_joints
    mujoco.mj_forward(model, data)

    # Viewer
    print("\n" + "=" * 60)
    print("Mink IK (Velocity Control)")
    print("Double-click and drag the RED box to command the robot!")
    print("=" * 60 + "\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        step_counter = 0
        last_stats_time = time.time()
        solve_times = []

        # Solver setup
        print(f"Available solvers: {qpsolvers.available_solvers}")
        solver = "daqp" if "daqp" in qpsolvers.available_solvers else "quadprog"
        print(f"Using solver: {solver}")

        while viewer.is_running():
            step_start = time.time()

            # Control rate: 1000Hz (Every step)
            # 1. Update Configuration from Real Robot (Sim)
            configuration.update(data.qpos)

            # 2. Get Target
            # Mink SE3 helper
            T_target = mink.SE3.from_mocap_name(model, data, "target")

            # 3. Update Tasks
            ee_task.set_target(T_target)

            # 4. Solve IK
            # dt = 0.001 (control step)
            try:
                solve_start = time.perf_counter()
                vel = mink.solve_ik(
                    configuration, tasks, dt=0.001, solver=solver, damping=1e-2
                )
                solve_end = time.perf_counter()
                solve_times.append(solve_end - solve_start)
            except Exception as e:
                # Fallback or print error
                # print(f"IK Failed: {e}")
                vel = np.zeros(model.nv)

            # 5. Apply Velocity
            # data.ctrl corresponds to the actuators.
            # Assuming actuators 0-6 are arm joints.
            # vel has size model.nv.
            data.ctrl[:7] = vel[:7]

            mujoco.mj_step(model, data)
            step_counter += 1
            viewer.sync()

            # Statistics
            if time.time() - last_stats_time >= 2.0:
                # Calculate Error
                # Mink can compute task error
                err_vec = ee_task.compute_error(configuration)  # 6D error
                pos_err = np.linalg.norm(err_vec[:3])

                # target_p = data.mocap_pos[0]

                if solve_times:
                    avg_solve_ms = np.mean(solve_times) * 1000
                    max_solve_ms = np.max(solve_times) * 1000
                    deadline_met = max_solve_ms < (model.opt.timestep * 1000)
                    solve_times = []  # Reset for next window
                else:
                    avg_solve_ms = 0
                    max_solve_ms = 0
                    deadline_met = False

                deadline_str = "OK" if deadline_met else "FAIL"

                print(
                    f"t:{data.time:.1f}s | Solve: {avg_solve_ms:.2f}ms (max: {max_solve_ms:.2f}ms) | Deadline: {deadline_str} | Error: {pos_err:.4f}m"
                )
                last_stats_time = time.time()

            # Timing
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
