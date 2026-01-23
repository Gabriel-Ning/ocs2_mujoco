#!/usr/bin/env python3
"""
Cartpole Swing-Up with OCS2 MPC and MuJoCo

This script demonstrates the integration of OCS2 (Optimal Control for Switched Systems)
with MuJoCo simulation. The key insight is that you only need THREE files:

1. model/task.yaml     - OCS2 problem formulation (costs, constraints, solver settings)
2. model/cartpole.urdf - Robot description (optional: for kinematics/dynamics)
3. model/cartpole.xml  - MuJoCo simulation model

The C++ interface (CartPoleInterface) reads task.yaml and sets up the optimal control
problem. Python bindings allow calling the MPC solver from this script.
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
    # Get example root directory (parent of scripts/)
    script_dir = os.path.dirname(os.path.abspath(__file__))
    example_dir = os.path.dirname(script_dir)

    # ==========================================================================
    # THE THREE ESSENTIAL FILES
    # ==========================================================================
    # 1. Task configuration: defines the OCS2 optimal control problem
    task_file = os.path.join(example_dir, "config/task.yaml")

    # 2. URDF file: robot kinematics (optional for cartpole, used for more complex robots)
    urdf_file = os.path.join(example_dir, "models/control/cartpole.urdf")

    # 3. MuJoCo XML: simulation model for physics and visualization
    mujoco_model = os.path.join(example_dir, "models/simulation/cartpole.xml")

    # Auto-generated CppAD code directory (for performance optimization)
    lib_folder = os.path.join(example_dir, "auto_generated")

    # Ensure auto_generated directory exists
    if not os.path.exists(lib_folder):
        os.makedirs(lib_folder)

    print("=" * 60)
    print("Cartpole Swing-Up Demo")
    print("=" * 60)
    print(f"Task file:     {task_file}")
    print(f"URDF file:     {urdf_file}")
    print(f"MuJoCo model:  {mujoco_model}")
    print(f"Library folder: {lib_folder}")
    print("=" * 60)

    # ==========================================================================
    # Initialize OCS2 MPC
    # ==========================================================================
    print("\nInitializing OCS2 MPC...")
    print("(First run compiles CppAD derivatives - this may take a moment)")
    try:
        mpc = CartpolePyBindings.mpc_interface(task_file, lib_folder, urdf_file)
    except Exception as e:
        print(f"Error initializing MPC: {e}")
        return 1

    # ==========================================================================
    # Initialize MuJoCo Simulation
    # ==========================================================================
    if not os.path.exists(mujoco_model):
        print(f"Error: {mujoco_model} not found.")
        return 1

    model = mujoco.MjModel.from_xml_path(mujoco_model)
    data = mujoco.MjData(model)

    # OCS2 solution buffers
    t_sol = CartpolePyBindings.scalar_array()
    x_sol = CartpolePyBindings.vector_array()
    u_sol = CartpolePyBindings.vector_array()

    # ==========================================================================
    # Set Initial and Target States
    # ==========================================================================
    # Initial state for MuJoCo: pole hanging down (theta = pi)
    # MuJoCo State vector: [cart_pos (x), pole_angle (theta), cart_vel, pole_angular_vel]
    initial_mujoco_state = np.array([0.0, np.pi, 0.0, 0.0])
    data.qpos[:] = initial_mujoco_state[:2]
    data.qvel[:] = initial_mujoco_state[2:]

    # Target state for OCS2: pole upright (theta = 0) at origin
    # OCS2 State vector: [theta, x, theta_dot, x_dot]
    target_ocs2_state = np.array([0.0, 0.0, 0.0, 0.0])

    t_target = CartpolePyBindings.scalar_array()
    x_target = CartpolePyBindings.vector_array()
    u_target = CartpolePyBindings.vector_array()

    t_target.push_back(0.0)
    x_target.push_back(target_ocs2_state)
    u_target.push_back(np.zeros(1))

    mpc.reset(CartpolePyBindings.TargetTrajectories(t_target, x_target, u_target))

    # ==========================================================================
    # Main Simulation Loop
    # ==========================================================================
    print("\nStarting simulation...")
    print("Goal: Swing the pole up from hanging position to upright")
    print("Press Ctrl+C or close the viewer to exit\n")

    # Frequencies
    mpc_freq = 100.0   # Hz (Run MPC at 100Hz)
    sim_dt = model.opt.timestep  # usually 0.001 (1kHz)
    mpc_dt = 1.0 / mpc_freq
    
    # Simulation state
    last_mpc_time = 0.0
    
    with mujoco.viewer.launch_passive(model, data) as viewer:
        start_time = time.time()
        while viewer.is_running():
            step_start = time.time()
            current_sim_time = data.time
            
            # Run MPC at specified frequency
            if current_sim_time - last_mpc_time >= mpc_dt:
                # Get current state from MuJoCo (x, theta, x_dot, theta_dot)
                qpos = data.qpos
                qvel = data.qvel
                
                # Map MuJoCo state to OCS2 state: [theta, x, theta_dot, x_dot]
                ocs2_state = np.array([qpos[1], qpos[0], qvel[1], qvel[0]])
    
                # Run MPC optimization
                mpc.setObservation(current_sim_time, ocs2_state, np.zeros(1))
                mpc.advanceMpc()
                mpc.getMpcSolution(t_sol, x_sol, u_sol)
                last_mpc_time = current_sim_time

            # Apply optimal control input
            # Note: OCS2 returns a trajectory, we just take the first element or interpolate
            if len(u_sol) > 0:
                data.ctrl[0] = u_sol[0][0]

            # Step physics
            mujoco.mj_step(model, data)

            # Update visualization
            viewer.sync()
            
            # Real-time synchronization
            # We want to keep the loop running at real-time speed relative to the simulation time
            # Since mj_step advances time by sim_dt, we want the wall clock to advance by sim_dt as well
            
            elapsed = time.time() - step_start
            sleep_time = sim_dt - elapsed
            if sleep_time > 0:
                time.sleep(sleep_time)

    return 0


if __name__ == "__main__":
    sys.exit(main())
