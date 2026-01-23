#!/usr/bin/env python3
"""
Cartpole Swing-Up with OCS2 MPC and MuJoCo
"""
import mujoco
import mujoco.viewer
import numpy as np
import os
import sys
import time

# Add pixi environment to path
sys.path.append(
    os.path.abspath(
        os.path.join(os.getcwd(), ".pixi/envs/default/lib/python3/dist-packages")
    )
)

from ocs2_cartpole import CartpolePyBindings


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # ==================================================================================
    # Step 1 & 3: Define paths to OCS2 resources (URDF and Task Configuration)
    # ==================================================================================
    task_file = os.path.join(script_dir, "config/mpc/task.info")
    urdf_file = os.path.join(script_dir, "urdf/cartpole.urdf")
    lib_folder = os.path.join(
        script_dir, "auto_generated/cartpole"
    )  # Place to store compiled auto-diff code

    # Check validity
    if not os.path.exists(lib_folder):
        os.makedirs(lib_folder)

    print("=" * 60)
    print("Cartpole Swing-Up Demo")
    print("=" * 60)

    # ==================================================================================
    # Step 4: Initialize the OCS2 MPC Interface
    # ==================================================================================
    print("Initializing MPC...")
    try:
        # This compiles the derivatives if they don't exist
        mpc = CartpolePyBindings.mpc_interface(task_file, lib_folder, urdf_file)
    except Exception as e:
        print(f"Error initializing MPC: {e}")
        return 1

    # ==================================================================================
    # Step 2: Load the MuJoCo Simulation Model
    # ==================================================================================
    model_path = os.path.join(script_dir, "cartpole.xml")
    if not os.path.exists(model_path):
        print(f"Error: {model_path} not found.")
        return 1

    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)

    # OCS2 solution buffers
    t_sol = CartpolePyBindings.scalar_array()
    x_sol = CartpolePyBindings.vector_array()
    u_sol = CartpolePyBindings.vector_array()

    # ==================================================================================
    # Step 5: Run the Simulation Loop
    # ==================================================================================

    # 5.1 Set Initial State (Pole down)
    initial_state = np.array([0.0, 3.14, 0.0, 0.0])  # [pos, angle, vel, ang_vel]
    data.qpos[:] = initial_state[:2]
    data.qvel[:] = initial_state[2:]

    # 5.2 Set Target Trajectory (Swing up to Upright)
    target_state = np.array([0.0, 0.0, 0.0, 0.0])

    t_target = CartpolePyBindings.scalar_array()
    x_target = CartpolePyBindings.vector_array()
    u_target = CartpolePyBindings.vector_array()

    t_target.push_back(0.0)
    x_target.push_back(target_state)
    u_target.push_back(np.zeros(1))

    mpc.reset(CartpolePyBindings.TargetTrajectories(t_target, x_target, u_target))

    print("Starting simulation...")
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            # --- State Estimation ---
            # Map MuJoCo state [qpos, qvel] to OCS2 state vector
            current_state = np.concatenate([data.qpos, data.qvel])

            # --- MPC Update ---
            mpc.setObservation(data.time, current_state, np.zeros(1))
            mpc.advanceMpc()
            mpc.getMpcSolution(t_sol, x_sol, u_sol)

            # --- Control Application ---
            if len(u_sol) > 0:
                # Apply optimal input to simulation
                u = u_sol[0][0]
                data.ctrl[0] = u

            # --- Physics Step ---
            mujoco.mj_step(model, data)

            # --- Visualization ---
            viewer.sync()
            time.sleep(model.opt.timestep)

    return 0


if __name__ == "__main__":
    main()
