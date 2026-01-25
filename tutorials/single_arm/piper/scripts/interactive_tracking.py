#!/usr/bin/env python3
"""
Interactive MPC Target Tracking for the Piper Robot
- MPC Control: Local URDF (6-DOF Arm)
- Simulation: Local MJCF
"""
import mujoco
import mujoco.viewer
import numpy as np
import os
import sys
import time

# Add pixi environment to path for OCS2 bindings
sys.path.append(
    os.path.abspath(
        os.path.join(os.getcwd(), ".pixi/envs/default/lib/python3/dist-packages")
    )
)

from ocs2_mobile_manipulator import MobileManipulatorPyBindings


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    # Paths
    task_file = os.path.join(script_dir, "../config/task.yaml")
    # We'll use a new auto-generated folder for piper
    lib_folder = os.path.join(script_dir, "../models/auto_generated")
    urdf_file = os.path.join(script_dir, "../models/control/urdf/piper.urdf")

    print("=" * 60)
    print("Piper Interactive MPC Tracking")
    print("=" * 60)

    # Load MuJoCo model (local model)
    local_mjcf_path = os.path.join(script_dir, "../models/simulation/scene.xml")
    model = mujoco.MjModel.from_xml_path(local_mjcf_path)
    data = mujoco.MjData(model)

    # Initialize OCS2 MPC
    try:
        mpc = MobileManipulatorPyBindings.mpc_interface(
            task_file, lib_folder, urdf_file
        )
    except Exception as e:
        print(f"ERROR initializing MPC: {e}")
        return 1

    # System dimensions
    state_dim = mpc.getStateDim()
    print(f"✓ OCS2 state dimension: {state_dim} (Should be 6 for arm)")

    # Solution arrays
    t_sol = MobileManipulatorPyBindings.scalar_array()
    x_sol = MobileManipulatorPyBindings.vector_array()
    u_sol = MobileManipulatorPyBindings.vector_array()

    # Initial target
    ee_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
    mujoco.mj_forward(model, data)
    # Get current EE pose in world frame
    ee_pos = data.site_xpos[ee_site_id].copy()
    ee_mat = data.site_xmat[
        ee_site_id
    ].copy()  # MuJoCo returns flat 9-element array for site_xmat
    ee_quat = np.zeros(4)
    mujoco.mju_mat2Quat(ee_quat, ee_mat)

    # Set mocap target to EE pose + small offset
    data.mocap_pos[0] = ee_pos + [0, 0, 0.1]
    data.mocap_quat[0] = ee_quat

    target_pos = data.mocap_pos[0].copy()
    target_quat_mj = data.mocap_quat[0].copy()
    target_quat_ocs2 = np.array(
        [target_quat_mj[1], target_quat_mj[2], target_quat_mj[3], target_quat_mj[0]]
    )
    target_state = np.concatenate([target_pos, target_quat_ocs2])

    t_target = MobileManipulatorPyBindings.scalar_array()
    x_target = MobileManipulatorPyBindings.vector_array()
    u_target = MobileManipulatorPyBindings.vector_array()
    t_target.push_back(0.0)
    x_target.push_back(target_state)
    u_target.push_back(np.zeros(state_dim))
    mpc.reset(
        MobileManipulatorPyBindings.TargetTrajectories(t_target, x_target, u_target)
    )

    # Control loop
    mpc_update_rate = 5  # Run MPC at every step (10ms)
    step_counter = 0
    last_u = np.zeros(state_dim)
    last_target_pos = target_pos.copy()
    mpc_solve_times = []
    last_stats_time = time.time()

    print("Opening MuJoCo viewer...")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            # Update target
            if step_counter % 20 == 0:  # Check target more frequently
                new_target_pos = data.mocap_pos[0].copy()
                if np.linalg.norm(new_target_pos - last_target_pos) > 0.005:
                    new_target_quat_mj = data.mocap_quat[0].copy()
                    new_target_quat_ocs2 = np.array(
                        [
                            new_target_quat_mj[1],
                            new_target_quat_mj[2],
                            new_target_quat_mj[3],
                            new_target_quat_mj[0],
                        ]
                    )
                    t_new = MobileManipulatorPyBindings.scalar_array()
                    x_new = MobileManipulatorPyBindings.vector_array()
                    u_new = MobileManipulatorPyBindings.vector_array()
                    t_new.push_back(data.time)
                    x_new.push_back(
                        np.concatenate([new_target_pos, new_target_quat_ocs2])
                    )
                    u_new.push_back(last_u)
                    mpc.reset(
                        MobileManipulatorPyBindings.TargetTrajectories(
                            t_new, x_new, u_new
                        )
                    )
                    last_target_pos = new_target_pos.copy()

            # MPC step
            if step_counter % mpc_update_rate == 0:
                mpc_start = time.time()
                mpc.setObservation(data.time, np.array(data.qpos[:state_dim]), last_u)
                mpc.advanceMpc()
                mpc.getMpcSolution(t_sol, x_sol, u_sol)
                mpc_solve_times.append((time.time() - mpc_start) * 1000)

                if len(u_sol) > 0:
                    current_u = np.array(u_sol[0])
                    data.ctrl[:6] = current_u[:6]
                    data.ctrl[6] = 0.1
                    last_u = current_u

            mujoco.mj_step(model, data)
            step_counter += 1

            if time.time() - last_stats_time >= 1.0 and len(mpc_solve_times) > 0:
                avg_solve = np.mean(mpc_solve_times)
                ee_pos = data.site_xpos[ee_site_id]
                error = np.linalg.norm(ee_pos - data.mocap_pos[0])
                ctrl = data.ctrl[:6]
                ctrl_str = ", ".join([f"{v:6.2f}" for v in ctrl])
                print(
                    f"t:{data.time:4.1f}s | MPC: {avg_solve:4.1f}ms | Error: {error:6.4f}m | U: [{ctrl_str}]"
                )
                mpc_solve_times.clear()
                last_stats_time = time.time()

            viewer.sync()
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    return 0


if __name__ == "__main__":
    exit(main())
