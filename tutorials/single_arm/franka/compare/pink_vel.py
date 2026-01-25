import mujoco
import mujoco.viewer
import numpy as np
import time
import pathlib

import pink
from pink import solve_ik
from pink.tasks import FrameTask, PostureTask
import pinocchio as pin


def main():
    # Paths
    current_dir = pathlib.Path(__file__).parent.absolute()
    model_dir = current_dir.parent / "models"
    urdf_path = model_dir / "control" / "urdf" / "panda.urdf"
    xml_path = model_dir / "simulation" / "scene.xml"

    # 1. Load MuJoCo Model (Simulation)
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    # 2. Load Pinocchio Model (Kinematics)
    # Use local meshes from 'models/control/meshes'
    pin_model = pin.buildModelFromUrdf(str(urdf_path))
    pin_data = pin_model.createData()

    # 3. Setup Pink Configuration
    q_ref = pin.neutral(pin_model)
    configuration = pink.Configuration(pin_model, pin_data, q_ref)

    # 4. Tasks
    ee_task = FrameTask(
        "panda_hand_tcp",
        position_cost=10.0,
        orientation_cost=10.0,
        lm_damping=1e-4,
    )

    posture_task = PostureTask(cost=1e-3)
    q_nom = q_ref.copy()
    if pin_model.nq >= 7:
        q_nom[:7] = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0])
    posture_task.set_target(q_nom)

    tasks = [ee_task, posture_task]

    # Initialization
    initial_joints = [0.0, 0.171, 0.114, -1.570, 0.050, 1.570, 0.469]
    data.qpos[:7] = initial_joints
    mujoco.mj_forward(model, data)

    # Viewer
    print("\n" + "=" * 60)
    print("Pink IK (Velocity Control)")
    print("Double-click and drag the RED box to command the robot!")
    print("=" * 60 + "\n")

    # Use daqp if available
    import qpsolvers

    print(f"Available solvers: {qpsolvers.available_solvers}")
    solver = "daqp" if "daqp" in qpsolvers.available_solvers else "quadprog"
    print(f"Using solver: {solver}")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        step_counter = 0
        last_stats_time = time.time()

        while viewer.is_running():
            step_start = time.time()

            # Control rate: 100Hz
            if step_counter % 10 == 0:
                dt_ctrl = 0.01

                # 1. Sync State: MuJoCo -> Pink
                q_pin = np.zeros(pin_model.nq)
                n_copy = min(len(data.qpos), pin_model.nq)
                q_pin[:n_copy] = data.qpos[:n_copy]
                configuration.update(q_pin)

                # 2. Get Target
                target_p = data.mocap_pos[0].copy()
                target_q = data.mocap_quat[0].copy()

                # Construct Pinocchio SE3 target (Quat: x,y,z,w)
                quat_pin = np.array(
                    [target_q[1], target_q[2], target_q[3], target_q[0]]
                )
                target_SE3 = pin.SE3(pin.Quaternion(quat_pin), target_p)

                ee_task.set_target(target_SE3)

                # 3. Solve IK
                try:
                    vel = solve_ik(configuration, tasks, dt_ctrl, solver=solver)
                except Exception:
                    vel = np.zeros(pin_model.nv)

                # 4. Apply Velocity
                if vel.shape[0] >= 7:
                    data.ctrl[:7] = vel[:7]

            mujoco.mj_step(model, data)
            step_counter += 1
            viewer.sync()

            # Statistics
            if time.time() - last_stats_time >= 2.0:
                ee_id = pin_model.getFrameId("panda_hand_tcp")
                H_curr = configuration.data.oMf[ee_id]
                p_curr = H_curr.translation
                target_p = data.mocap_pos[0]
                pos_err = np.linalg.norm(target_p - p_curr)

                print(
                    f"t:{data.time:.1f}s | Pink Rate: 100Hz | Target: [{target_p[0]:.2f}, {target_p[1]:.2f}, {target_p[2]:.2f}] | Error: {pos_err:.4f}m"
                )
                last_stats_time = time.time()

            # Timing
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
