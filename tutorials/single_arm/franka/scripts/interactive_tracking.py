#!/usr/bin/env python3
"""
Interactive MPC Target Tracking with MuJoCo Simulation

This example demonstrates the OCS2 MPC Python interface for real-time robot control:
- Drag the RED mocap body to command the robot's end-effector target
- The robot will track the target position and orientation in real-time
- Uses the MPC_MRT_Interface pattern for synchronous MPC execution

Architecture:
    MuJoCo Simulator <-> Python Script <-> OCS2 MPC (C++ via pybind11)

The script implements a basic MPC control loop:
    1. Read current state from simulation
    2. Update target from mocap body
    3. Run MPC to compute optimal control
    4. Apply control to simulation
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


def verify_yaml_config(task_file):
    """Verify the YAML configuration file can be parsed correctly."""
    try:
        import yaml

        with open(task_file, "r") as f:
            config = yaml.safe_load(f)

        mi = config.get("model_information", {})
        remove_joints = mi.get("removeJoints", [])
        manipulator_type = mi.get("manipulatorModelType", -1)
        base_frame = mi.get("baseFrame", "")
        ee_frame = mi.get("eeFrame", "")

        print(f"  Manipulator type: {manipulator_type}")
        print(f"  Base frame: {base_frame}")
        print(f"  EE frame: {ee_frame}")
        print(f"  Remove joints: {remove_joints}")

        if not remove_joints:
            print(
                "  WARNING: No joints marked for removal - URDF may have wrong DOF count"
            )

        return True
    except Exception as e:
        print(f"  ERROR parsing YAML: {e}")
        return False


def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))

    # Paths - can use either .info or .yaml format
    model_path = os.path.join(script_dir, "../models/simulation/scene.xml")
    task_file = os.path.join(script_dir, "../config/task.yaml")
    lib_folder = os.path.join(script_dir, "../models/auto_generated")
    urdf_file = os.path.join(script_dir, "../models/control/urdf/panda.urdf")

    print("=" * 60)
    print("Interactive MPC Tracking - Drag the Target!")
    print("=" * 60)

    # Verify files exist
    for name, path in [
        ("Task file", task_file),
        ("URDF file", urdf_file),
        ("MuJoCo model", model_path),
    ]:
        if not os.path.exists(path):
            print(f"ERROR: {name} not found: {path}")
            return 1
        print(f"{name}: {path}")

    # Verify YAML configuration
    print("\nVerifying configuration...")
    if not verify_yaml_config(task_file):
        return 1

    # Load MuJoCo model
    print(f"\nLoading MuJoCo model...")
    model = mujoco.MjModel.from_xml_path(model_path)
    data = mujoco.MjData(model)
    print(f"  MuJoCo DOF: {model.nq}")

    # Initialize OCS2 MPC
    print(f"\nInitializing OCS2 MPC...")
    print(f"  Library folder: {lib_folder}")

    try:
        mpc = MobileManipulatorPyBindings.mpc_interface(
            task_file, lib_folder, urdf_file
        )
    except Exception as e:
        print(f"ERROR initializing MPC: {e}")
        print("\nThis usually means:")
        print(
            "  1. The URDF has more DOF than expected (check removeJoints in task.yaml)"
        )
        print("  2. The task file format is incorrect")
        print("  3. The library folder is missing compiled libraries")
        return 1

    # Get system dimensions from MPC
    state_dim = mpc.getStateDim()
    input_dim = mpc.getInputDim()
    print(f"  OCS2 state dimension: {state_dim}")
    print(f"  OCS2 input dimension: {input_dim}")

    # Verify dimensions match
    if state_dim != 7:
        print(f"  WARNING: Expected 7 DOF but got {state_dim}")
        print("  Check that removeJoints in task.yaml correctly removes finger joints")

    # Create solution arrays
    t_sol = MobileManipulatorPyBindings.scalar_array()
    x_sol = MobileManipulatorPyBindings.vector_array()
    u_sol = MobileManipulatorPyBindings.vector_array()
    print("  MPC initialized successfully")

    # Set initial state
    initial_joints = [0.0, 0.171, 0.114, -1.570, 0.050, 1.570, 0.469]
    data.qpos[:] = initial_joints
    data.qvel[:] = [0.0] * 7

    # Get IDs
    try:
        ee_site_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SITE, "ee_site")
        print(f"✓ End-effector site found (ID: {ee_site_id})")
    except:
        ee_site_id = -1
        print("✗ Warning: ee_site not found")

    # Initial MPC reset with current mocap position
    target_pos = data.mocap_pos[0].copy()
    target_quat_mj = data.mocap_quat[0].copy()
    # Convert MuJoCo quat [w,x,y,z] to [x,y,z,w]
    target_quat = np.array(
        [target_quat_mj[1], target_quat_mj[2], target_quat_mj[3], target_quat_mj[0]]
    )
    target_state = np.concatenate([target_pos, target_quat])

    t_target = MobileManipulatorPyBindings.scalar_array()
    x_target = MobileManipulatorPyBindings.vector_array()
    u_target = MobileManipulatorPyBindings.vector_array()
    t_target.push_back(0.0)
    x_target.push_back(target_state)
    u_target.push_back(np.zeros(7))
    target_traj = MobileManipulatorPyBindings.TargetTrajectories(
        t_target, x_target, u_target
    )
    mpc.reset(target_traj)
    print(f"✓ Initial target: {target_pos}")

    # Control config
    mpc_update_rate = 5  # Update MPC every 10 steps (100Hz)
    target_update_rate = 50  # Update target every 50 steps (20Hz)
    step_counter = 0
    last_u = np.zeros(7)

    # Tracking
    last_stats_time = time.time()
    last_target_pos = target_pos.copy()
    mpc_solve_times = []

    print("\n" + "=" * 60)
    print("INTERACTIVE MODE")
    print("=" * 60)
    print("🔴 RED box with RGB axes = Draggable target")
    print("🟢 GREEN sphere = Old default target (ignore)")
    print("\nDouble-click and drag the RED box to command the robot!")
    print("The robot will track the target in real-time")
    print("=" * 60 + "\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            step_start = time.time()

            # Update target from mocap periodically
            if step_counter % target_update_rate == 0:
                # Read current mocap pose
                new_target_pos = data.mocap_pos[0].copy()
                new_target_quat_mj = data.mocap_quat[0].copy()
                new_target_quat = np.array(
                    [
                        new_target_quat_mj[1],
                        new_target_quat_mj[2],
                        new_target_quat_mj[3],
                        new_target_quat_mj[0],
                    ]
                )

                # Check if target moved significantly (>1cm)
                if np.linalg.norm(new_target_pos - last_target_pos) > 0.01:
                    # Update MPC target by resetting with new target
                    new_target_state = np.concatenate([new_target_pos, new_target_quat])

                    t_new = MobileManipulatorPyBindings.scalar_array()
                    x_new = MobileManipulatorPyBindings.vector_array()
                    u_new = MobileManipulatorPyBindings.vector_array()
                    t_new.push_back(data.time)
                    x_new.push_back(new_target_state)
                    u_new.push_back(last_u)

                    try:
                        new_traj = MobileManipulatorPyBindings.TargetTrajectories(
                            t_new, x_new, u_new
                        )
                        mpc.reset(new_traj)
                        last_target_pos = new_target_pos.copy()
                        print(
                            f"→ New target: [{new_target_pos[0]:.3f}, {new_target_pos[1]:.3f}, {new_target_pos[2]:.3f}]"
                        )
                    except Exception as e:
                        print(f"Target update error: {e}")

            # MPC update
            if step_counter % mpc_update_rate == 0:
                t_curr = data.time
                ocs2_x = np.array(data.qpos[:7])

                mpc_start = time.time()
                try:
                    mpc.setObservation(t_curr, ocs2_x, last_u)
                    mpc.advanceMpc()
                    mpc.getMpcSolution(t_sol, x_sol, u_sol)

                    mpc_solve_time = (time.time() - mpc_start) * 1000
                    mpc_solve_times.append(mpc_solve_time)

                    if len(u_sol) > 0:
                        last_u = np.array(u_sol[0])
                except Exception as e:
                    print(f"MPC error: {e}")

            # Apply control
            ctrl = np.clip(last_u, -2.61, 2.61)
            data.ctrl[:] = ctrl

            # Step physics
            mujoco.mj_step(model, data)
            step_counter += 1

            # Print stats every 2 seconds
            if time.time() - last_stats_time >= 2.0 and len(mpc_solve_times) > 0:
                avg_solve = np.mean(mpc_solve_times)

                # Get current state
                current_target = data.mocap_pos[0]
                if ee_site_id >= 0:
                    mujoco.mj_forward(model, data)
                    ee_pos = data.site_xpos[ee_site_id]
                    error = np.linalg.norm(current_target - ee_pos)
                    print(
                        f"t:{data.time:.1f}s | MPC: {avg_solve:.1f}ms | "
                        f"Target: [{current_target[0]:.2f}, {current_target[1]:.2f}, {current_target[2]:.2f}] | "
                        f"Error: {error:.3f}m"
                    )
                else:
                    print(f"t:{data.time:.1f}s | MPC: {avg_solve:.1f}ms")

                mpc_solve_times.clear()
                last_stats_time = time.time()

            # Sync viewer
            viewer.sync()

            # Real-time pacing
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)

    print("\n" + "=" * 60)
    print("Interactive session ended")
    print("=" * 60)
    return 0


if __name__ == "__main__":
    exit(main())
