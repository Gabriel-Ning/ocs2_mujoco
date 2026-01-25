import mujoco
import mujoco.viewer
import numpy as np
import casadi as cs
import os
import sys
import time
import pathlib
import scipy.spatial.transform

# Add adam to python path
current_dir = pathlib.Path(__file__).parent.absolute()
adam_path = current_dir.parent.parent.parent.parent / "adam" / "src"
sys.path.append(str(adam_path))

from adam.casadi import KinDynComputations


class InverseKinematics:
    def __init__(self, kindyn, ee_frame):
        self.kindyn = kindyn
        self.ee_frame = ee_frame

        # Setup CasADi functions
        self.q_sym = cs.SX.sym("q", 7)
        self.base_H = np.eye(4)

        # Forward Kinematics
        self.fk_fun = kindyn.forward_kinematics_fun(ee_frame)
        self.J_fun = kindyn.jacobian_fun(ee_frame)

    def solve(self, q_curr, p_des, R_des, v_err_int=None, w_err_int=None):
        # 1. Forward Kinematics
        H_curr = self.fk_fun(self.base_H, q_curr)
        # Convert DM/SX to numpy
        H_curr = np.array(H_curr)
        p_curr = H_curr[:3, 3]
        R_curr = H_curr[:3, :3]

        # 2. Compute Error Twist (6D) in Base Frame
        # Position Error
        v_err = p_des - p_curr

        # Orientation Error
        try:
            R_rel = R_des @ R_curr.T
            rot = scipy.spatial.transform.Rotation.from_matrix(R_rel)
            w_err = rot.as_rotvec()
        except ValueError:
            w_err = np.zeros(3)

        # Combined Error
        # Gains
        Kp_pos = 10.0
        Kp_rot = 10.0
        Ki = 2.0  # Strong Integral Gain

        V_cmd = np.concatenate([Kp_pos * v_err, Kp_rot * w_err])

        # Add Integral Term
        if v_err_int is not None and w_err_int is not None:
            V_cmd += np.concatenate([Ki * v_err_int, Ki * w_err_int])

        J = np.array(self.J_fun(self.base_H, q_curr))
        # Adam returns full Jacobian (including base 6DOF if detected as floating).
        # We assume base is fixed, so we take the last 7 columns corresponding to joints.
        if J.shape[1] == 13:
            J = J[:, 6:]
        # J is now 6x7

        # 4. Damped Pseudo Inverse
        lam = 1e-4
        lambda_sq = lam**2
        try:
            J_pinv = J.T @ np.linalg.inv(J @ J.T + lambda_sq * np.eye(6))
        except np.linalg.LinAlgError:
            J_pinv = np.linalg.pinv(J)

        # 5. Nullspace Biasing
        q_nom = np.array([0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0])  # Comfortable pose
        q_bias = -0.05 * (q_curr - q_nom)  # Weak Pull towards nominal

        P_null = np.eye(7) - J_pinv @ J

        q_dot = J_pinv @ V_cmd + P_null @ q_bias

        return q_dot, v_err, w_err


def main():
    # Paths
    current_dir = pathlib.Path(__file__).parent.absolute()
    model_dir = current_dir.parent / "models"
    urdf_path = model_dir / "control" / "urdf" / "panda.urdf"
    xml_path = model_dir / "simulation" / "scene.xml"

    # Load MuJoCo
    model = mujoco.MjModel.from_xml_path(str(xml_path))
    data = mujoco.MjData(model)

    # Setup Adam
    with open(urdf_path, "r") as f:
        urdf_string = f.read()

    joints_name_list = [
        "panda_joint1",
        "panda_joint2",
        "panda_joint3",
        "panda_joint4",
        "panda_joint5",
        "panda_joint6",
        "panda_joint7",
    ]
    kindyn = KinDynComputations(
        urdfstring=urdf_string, joints_name_list=joints_name_list
    )

    ik_solver = InverseKinematics(kindyn, "panda_hand_tcp")

    # DEBUG: Check if TCP offset is respected
    fk_hand = kindyn.forward_kinematics_fun("panda_hand")(np.eye(4), [0] * 7)
    fk_tcp = kindyn.forward_kinematics_fun("panda_hand_tcp")(np.eye(4), [0] * 7)
    diff = np.linalg.norm(np.array(fk_hand)[:3, 3] - np.array(fk_tcp)[:3, 3])
    print(f"DEBUG: Distance between 'panda_hand' and 'panda_hand_tcp': {diff:.4f}m")

    # Initialization
    initial_joints = [0.0, 0.171, 0.114, -1.570, 0.050, 1.570, 0.469]
    data.qpos[:7] = initial_joints

    # Integral Error State
    v_err_int = np.zeros(3)
    w_err_int = np.zeros(3)
    dt_ctrl = 0.01

    # Viewer
    print("\n" + "=" * 60)
    print("Adam Analytical IK (Velocity Control + Integral)")
    print("Double-click and drag the RED box to command the robot!")
    print("=" * 60 + "\n")

    with mujoco.viewer.launch_passive(model, data) as viewer:
        step_counter = 0
        last_stats_time = time.time()
        while viewer.is_running():
            step_start = time.time()

            # Control rate: 100Hz
            if step_counter % 10 == 0:
                # 1. State
                q_curr = data.qpos[:7].copy()

                # 2. Target
                target_p = data.mocap_pos[0].copy()
                target_q = data.mocap_quat[0].copy()
                target_R = scipy.spatial.transform.Rotation.from_quat(
                    [target_q[1], target_q[2], target_q[3], target_q[0]]
                ).as_matrix()  # MuJoCo w,x,y,z -> Scipy x,y,z,w

                # 3. Solve IK
                q_dot_cmd, v_err, w_err = ik_solver.solve(
                    q_curr, target_p, target_R, v_err_int, w_err_int
                )

                # Integrate Error
                v_err_int += v_err * dt_ctrl
                w_err_int += w_err * dt_ctrl

                # Anti-windup
                v_err_int = np.clip(v_err_int, -0.2, 0.2)
                w_err_int = np.clip(w_err_int, -0.2, 0.2)

                # 4. Limit Velocity
                v_max = 2.0
                q_dot_cmd = np.clip(q_dot_cmd, -v_max, v_max)

                # 5. Apply
                data.ctrl[:7] = q_dot_cmd

            mujoco.mj_step(model, data)
            step_counter += 1
            viewer.sync()

            # Statistics
            if time.time() - last_stats_time >= 2.0:
                # Calculate current error for display
                H_curr = np.array(ik_solver.fk_fun(np.eye(4), data.qpos[:7]))
                p_curr = H_curr[:3, 3]
                target_p = data.mocap_pos[0]
                pos_err = np.linalg.norm(target_p - p_curr)

                print(
                    f"t:{data.time:.1f}s | IK Rate: 100Hz | Target: [{target_p[0]:.2f}, {target_p[1]:.2f}, {target_p[2]:.2f}] | Error: {pos_err:.4f}m"
                )
                last_stats_time = time.time()

            # Timing
            time_until_next_step = model.opt.timestep - (time.time() - step_start)
            if time_until_next_step > 0:
                time.sleep(time_until_next_step)


if __name__ == "__main__":
    main()
