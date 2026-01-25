import mujoco
import mujoco.viewer
import numpy as np
import casadi as cs
import os
import sys
import time
import pathlib
from adam.casadi import KinDynComputations


def main():
    # Paths
    current_dir = pathlib.Path(__file__).parent.absolute()
    model_dir = current_dir.parent / "models"
    urdf_path = model_dir / "control" / "urdf" / "panda.urdf"
    xml_path = model_dir / "simulation" / "scene.xml"

    if not urdf_path.exists():
        print(f"Error: URDF not found at {urdf_path}")
        return
    if not xml_path.exists():
        print(f"Error: XML not found at {xml_path}")
        return

    # Load MuJoCo model
    print(f"Loading MuJoCo model from {xml_path}")
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    # Read URDF for Adam
    with open(urdf_path, "r") as f:
        urdf_string = f.read()

    # Adam KinDyn setup
    joints_name_list = [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    ]

    ee_frame = "panda_hand_tcp"

    print("Initializing Adam KinDynComputations...")
    kindyn = KinDynComputations(
        urdfstring=urdf_string, joints_name_list=joints_name_list
    )

    # --- MPC Formulation ---
    print("Setting up CasADi MPC...")
    opti = cs.Opti()
    N = 50  # Horizon length: 50 * 0.02 = 1.0s
    dt = 0.02  # Time step for MPC (Finer resolution for comparison)
    # Note: OCS2 uses 1ms integration. We use 20ms for NLP real-time feasibility.

    n_q = 7
    q = opti.variable(n_q, N + 1)
    q_dot = opti.variable(n_q, N)

    q_current = opti.parameter(n_q)
    target_pos = opti.parameter(3)
    target_rot = opti.parameter(3, 3)  # Target rotation matrix

    # Initial state constraint
    opti.subject_to(q[:, 0] == q_current)

    # Dynamics (Integration)
    for i in range(N):
        opti.subject_to(q[:, i + 1] == q[:, i] + q_dot[:, i] * dt)

    # Velocity limits (approximate values from URDF)
    # panda joints: 2.1750, 2.1750, 2.1750, 2.1750, 2.6100, 2.6100, 2.6100
    v_max = np.array([2.175, 2.175, 2.175, 2.175, 2.61, 2.61, 2.61])
    # Apply limits
    # opti.bounded is element-wise? No, usually separate.
    opti.subject_to(opti.bounded(-v_max.reshape(-1, 1), q_dot, v_max.reshape(-1, 1)))

    # Cost Function
    fk_fun = kindyn.forward_kinematics_fun(ee_frame)
    base_H = np.eye(4)

    cost = 0

    # Running cost
    for i in range(1, N + 1):
        H_ee = fk_fun(base_H, q[:, i])
        pos_ee = H_ee[:3, 3]
        rot_ee = H_ee[:3, :3]

        # Position error
        cost += cs.sumsqr(pos_ee - target_pos) * 50.0

        # Orientation error (Frobenius norm of rotation matrix difference)
        # ||R_ee - R_target||_F^2 = tr((R_ee - R_tgt).T @ (R_ee - R_tgt))
        # Equivalent to 2*(3 - tr(R_ee.T @ R_tgt))
        # We use a simple sumsqr element-wise diff which is the squared Frobenius norm
        cost += cs.sumsqr(rot_ee - target_rot) * 10.0

        # Velocity regularization
        if i <= N:
            cost += cs.sumsqr(q_dot[:, i - 1]) * 0.01

    opti.minimize(cost)

    # Solver options
    # jit = True can speed up but requires compiler
    opts = {
        "ipopt.print_level": 0,
        "print_time": 0,
        "ipopt.sb": "yes",
        "expand": True,
        "ipopt.max_iter": 20,  # Limit iterations for real-time
        "ipopt.tol": 1e-4,
    }
    opti.solver("ipopt", opts)  # Using IPOPT

    # Initialization
    initial_joints = [0.0, 0.171, 0.114, -1.570, 0.050, 1.570, 0.469]
    data.qpos[:7] = initial_joints
    data.qvel[:7] = 0.0

    # Get ID for stats
    try:
        ee_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
    except:
        ee_site_id = -1

    # Loop vars
    mpc_decimation = 5  # Run MPC every 5 sim steps (5ms = 200Hz)
    # OCS2 ran at 200Hz (every 5 steps). IPOPT might be too slow for 200Hz.
    # Let's try 50Hz (20ms budget).
    step_counter = 0

    q_sol = np.tile(np.array(initial_joints).reshape(7, 1), (1, N + 1))
    q_dot_sol = np.zeros((n_q, N))
    last_u = np.zeros(n_q)

    mpc_solve_times = []
    last_stats_time = time.time()

    print("\n" + "=" * 60)
    print("Adam + CasADi Kinematics MPC (6D Pose Tracking)")
    print("Double-click and drag the RED box to command the robot!")
    print("=" * 60 + "\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            # --- MPC Step ---
            if step_counter % mpc_decimation == 0:
                mpc_start = time.time()

                # Get State
                q_meas = data.qpos[:7].copy()

                # Get Target
                if model.nmocap > 0:
                    target_p = data.mocap_pos[0].copy()
                    target_q = data.mocap_quat[0].copy()  # [w, x, y, z]

                    # Convert quaternion to rotation matrix
                    # start with identity
                    target_R_mat = np.zeros((3, 3))
                    # Manual quat2mat (w, x, y, z)
                    w, x, y, z = target_q
                    target_R_mat[0, 0] = 1 - 2 * (y**2 + z**2)
                    target_R_mat[0, 1] = 2 * (x * y - z * w)
                    target_R_mat[0, 2] = 2 * (x * z + y * w)
                    target_R_mat[1, 0] = 2 * (x * y + z * w)
                    target_R_mat[1, 1] = 1 - 2 * (x**2 + z**2)
                    target_R_mat[1, 2] = 2 * (y * z - x * w)
                    target_R_mat[2, 0] = 2 * (x * z - y * w)
                    target_R_mat[2, 1] = 2 * (y * z + x * w)
                    target_R_mat[2, 2] = 1 - 2 * (x**2 + y**2)
                else:
                    target_p = np.array([0.5, 0.0, 0.5])
                    target_R_mat = np.eye(3)

                try:
                    # Set params
                    opti.set_value(q_current, q_meas)
                    opti.set_value(target_pos, target_p)
                    opti.set_value(target_rot, target_R_mat)

                    # Warm start
                    opti.set_initial(q, q_sol)
                    opti.set_initial(q_dot, q_dot_sol)

                    # Solve
                    sol = opti.solve()

                    # Store sol
                    q_sol = sol.value(q)
                    q_dot_sol = sol.value(q_dot)

                    # Control: 1st velocity
                    v_cmd = q_dot_sol[:, 0]
                    last_u = v_cmd

                except Exception as e:
                    # If solve fails (e.g. max iter), usually we can still get value?
                    # Or opti.debug.value(q_dot)
                    # print(f"MPC Fail: {e}")
                    # Keep last u or zero?
                    # last_u = np.zeros(n_q)
                    pass

                mpc_dur = (time.time() - mpc_start) * 1000
                mpc_solve_times.append(mpc_dur)

            # Apply Control
            # Note: CasADi computes desired velocity q_dot.
            # We treat this as the command to the velocity actuator.
            data.ctrl[:7] = last_u

            # Statistics
            if time.time() - last_stats_time >= 2.0:
                avg_solve = np.mean(mpc_solve_times) if mpc_solve_times else 0.0

                current_target = data.mocap_pos[0]
                if ee_site_id >= 0:
                    # We need updated forward kinematics for the current state to check error
                    # mujoco.mj_forward(model, data) # step() does this usually?
                    # Actually step() advances state. After step, data.site_xpos is valid?
                    # Yes, but we check before or after step?
                    # Best to check current state.
                    ee_pos = data.site_xpos[ee_site_id]
                    error = np.linalg.norm(current_target - ee_pos)
                    print(
                        f"t:{data.time:.1f}s | MPC: {avg_solve:.1f}ms | "
                        f"Target: [{current_target[0]:.2f}, {current_target[1]:.2f}, {current_target[2]:.2f}] | "
                        f"Error: {error:.3f}m"
                    )
                else:
                    print(f"t:{data.time:.1f}s | MPC: {avg_solve:.1f}ms")

                mpc_solve_times = []
                last_stats_time = time.time()

            # Step Physics
            mujoco.mj_step(model, data)
            step_counter += 1

            viewer.sync()

            # Timing
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
