#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os.path
from dataclasses import dataclass
from enum import Enum

import matplotlib.pyplot as plt
import numpy as np
import sympy as sp
from scipy.spatial.transform import Rotation

DISABLE_LOGGING = False


class bcolors:
    HEADER = "\033[95m"
    OKBLUE = "\033[94m"
    OKCYAN = "\033[96m"
    OKGREEN = "\033[92m"
    WARNING = "\033[93m"
    FAIL = "\033[91m"
    ENDC = "\033[0m"
    BOLD = "\033[1m"
    UNDERLINE = "\033[4m"


class Action(Enum):
    MOVE = 0
    GRIP = 1


@dataclass
class JointAction:
    action: Action

    # if action == MOVE
    position: list[float] | None = None
    orientation: list[float] | None = None

    # if action == GRIP
    force: float | None = None

    def __post_init__(self):
        if self.action == Action.MOVE:
            if (self.position is None or self.orientation is None) and not (
                self.position is None and self.orientation is None
            ):
                raise ValueError(
                    "Position and orientation must both be None or list[float] for MOVE action."
                )
        elif self.action == Action.GRIP:
            if self.force is None:
                raise ValueError("Force must be provided for GRIP action.")


JOINT_MIN_LIMITS = [-3.14, -2.04, -2.61, -3.14, -1.04, -3.14]
JOINT_MAX_LIMITS = [3.14, 2.04, 2.61, 3.14, 2.56, 3.14]

END_EFFECTOR_ATTACHED = True
ITER_COUNT = 100

DH_a = [0, -0.266, 0, 0, 0, 0]  # link lengths in meters
DH_alpha = [90, 180, 90, -90, 90, 0]  # link twists in degrees
DH_d = [
    0.1925,
    0,
    0,
    0.324,
    0,
    0.155 + (0.219 / 2 if END_EFFECTOR_ATTACHED else 0),
]  # link offsets in meters (half the last link length)

Kp_pos = 0.4  # proportional gain for position
Kd_pos = 0.1  # derivative gain for position

Kp_rot = np.deg2rad(10.0)  # rotational proportional gain
Kd_rot = np.deg2rad(0.05)  # rotational derivative gain

pos_tolerance = 1e-6
rot_tolerance = 1e-3


class MoveSolver:
    def __init__(self, node):
        self.node = node

        # Joint angles (theta_i) - all are variables
        self.theta1, self.theta2, self.theta3, self.theta4, self.theta5, self.theta6 = (
            sp.symbols("theta1 theta2 theta3 theta4 theta5 theta6")
        )
        self.theta = [
            self.theta1,
            self.theta2 - sp.pi / 2,
            self.theta3 - sp.pi / 2,
            self.theta4,
            self.theta5,
            self.theta6,
        ]

        self.A = []
        self.R = None
        self.p = None
        self.Jv = sp.zeros(3, 6)
        self.Jw = sp.zeros(3, 6)
        self.J = sp.zeros(6, 6)

        # transform matrices from base to end-effector
        self.T = None
        self.T06 = None

        self.calc_jacobian()

    def log(self, message):
        if not DISABLE_LOGGING:
            self.node.get_logger().info(message)

    def calc_jacobian(self):
        for i in range(6):
            self.A.append(
                sp.Matrix(
                    [
                        [
                            sp.cos(self.theta[i]),
                            -sp.sin(self.theta[i]) * np.cos(np.deg2rad(DH_alpha[i])),
                            sp.sin(self.theta[i]) * np.sin(np.deg2rad(DH_alpha[i])),
                            DH_a[i] * sp.cos(self.theta[i]),
                        ],
                        [
                            sp.sin(self.theta[i]),
                            sp.cos(self.theta[i]) * np.cos(np.deg2rad(DH_alpha[i])),
                            -sp.cos(self.theta[i]) * np.sin(np.deg2rad(DH_alpha[i])),
                            DH_a[i] * sp.sin(self.theta[i]),
                        ],
                        [
                            0,
                            np.sin(np.deg2rad(DH_alpha[i])),
                            np.cos(np.deg2rad(DH_alpha[i])),
                            DH_d[i],
                        ],
                        [0, 0, 0, 1],
                    ]
                )
            )

        self.T = []
        self.T06 = self.A[0]
        self.T.append(self.T06)
        for i in range(1, 6):
            self.T06 = self.T06 * self.A[i]
            self.T.append(self.T06)

        self.R = self.T06[:3, :3]
        self.p = self.T06[:3, 3]

        self.Jv = sp.zeros(3, 6)
        self.Jw = sp.zeros(3, 6)

        T_prev = sp.eye(4)
        z_prev = sp.Matrix([0, 0, 1])
        p_prev = sp.Matrix([0, 0, 0])
        for i in range(6):
            T_prev = T_prev * self.A[i]
            z_curr = T_prev[:3, 2]
            p_curr = T_prev[:3, 3]
            self.Jv[:, i] = z_prev.cross(self.p - p_prev)
            self.Jw[:, i] = z_prev
            z_prev = z_curr
            p_prev = p_curr

        # Combine Jacobian components
        self.J = sp.Matrix.vstack(self.Jv, self.Jw)

    def pose_from_joints(self, joints):
        joints = np.array(joints, dtype=float)
        position = np.array(
            self.p.subs(
                [
                    (self.theta1, joints[0]),
                    (self.theta2, joints[1]),
                    (self.theta3, joints[2]),
                    (self.theta4, joints[3]),
                    (self.theta5, joints[4]),
                    (self.theta6, joints[5]),
                ]
            ),
            dtype=float,
        )
        position = np.round(position, 4).flatten()
        rotation_matrix = np.array(
            self.R.subs(
                [
                    (self.theta1, joints[0]),
                    (self.theta2, joints[1]),
                    (self.theta3, joints[2]),
                    (self.theta4, joints[3]),
                    (self.theta5, joints[4]),
                    (self.theta6, joints[5]),
                ]
            ),
            dtype=float,
        )
        orientation = Rotation.from_matrix(rotation_matrix).as_quat()
        orientation = np.round(orientation, 4)
        return position.tolist(), orientation.tolist()

    def check_pose(self, drag_joints, target_pose, target_orientation, retry=False):
        self.log(f"Checking pose: {drag_joints}")
        drag_joints = np.mod(drag_joints, 2 * np.pi)
        drag_joints = np.array(
            [(j - 2 * np.pi) if j > np.pi else j for j in drag_joints], dtype=float
        )
        drag_joints = np.round(drag_joints, 2)
        self.log(
            f"Pose re-mapped to [-pi, pi] range: {'' if retry else bcolors.OKGREEN} {drag_joints} {'' if retry else bcolors.ENDC}"
        )
        for i in range(6):
            transform = np.array(
                self.T[i].subs(
                    [
                        (self.theta1, drag_joints[0]),
                        (self.theta2, drag_joints[1]),
                        (self.theta3, drag_joints[2]),
                        (self.theta4, drag_joints[3]),
                        (self.theta5, drag_joints[4]),
                        (self.theta6, drag_joints[5]),
                    ]
                ),
                dtype=float,
            )
            final_p = np.round(transform[:3, 3], 4)
            self.log(f"Position {i}: {final_p}")
            if (
                final_p[0] > 0.48 or final_p[2] < 0.1
            ):  # compare to workspace limits (i.e., floor and wall)
                self.log(f"Position {i} is invalid")
                if i >= 3:
                    self.log("Target position/orientation is invalid")
                    return False, drag_joints
                elif i >= 1 and not retry:
                    self.log("Trying to fix via the other triangle solution")
                    # Adjust joints 2, 3, and 5 to switch to the other elbow configuration
                    # joint 2 = joint 2 - joint 3
                    # joint 3 = -joint 3
                    # joint 5 = joint 5 + joint 3
                    drag_joints[1] -= drag_joints[2]
                    drag_joints[4] += drag_joints[2]
                    drag_joints[2] *= -1

                    drag_joints = self.compute(
                        target_pose, target_orientation, drag_joints, retry=True
                    )
                    return self.check_pose(
                        drag_joints, target_pose, target_orientation, retry=True
                    )
                else:
                    self.log("Unknown error")
                    return False, drag_joints
            elif drag_joints[i] != np.clip(
                drag_joints[i], JOINT_MIN_LIMITS[i], JOINT_MAX_LIMITS[i]
            ):
                self.log(f"Joint {i} is invalid")
                if not retry:
                    self.log("Trying to fix via the other triangle solution")
                    drag_joints[1] -= drag_joints[2]
                    drag_joints[4] += drag_joints[2]
                    drag_joints[2] *= -1

                    drag_joints = self.compute(
                        target_pose, target_orientation, drag_joints, retry=True
                    )
                    return self.check_pose(
                        drag_joints, target_pose, target_orientation, retry=True
                    )
                else:
                    self.log("Unknown error")
                    return False, drag_joints

        return True, drag_joints

        # position 1/2 == joint 3
        # position 3/4 == joint 5
        # position 5 == target position

    def compute(
        self, target_pose, target_orientation, starting_joint_state, retry=False
    ):
        theta_vals = np.array(starting_joint_state, dtype=float)
        target_pose = np.array(target_pose, dtype=float)
        target_orientation = np.array(target_orientation, dtype=float)
        target_orientation_matrix = Rotation.from_euler(
            "xyz", target_orientation, degrees=True
        ).as_matrix()

        position_errors = []
        orientation_errors = []

        for iter in range(ITER_COUNT):
            if iter % 10 == 0 or iter == ITER_COUNT - 1:
                self.log("Iteration: {}".format(iter))

            # Current end-effector position and orientation
            current_position = np.array(
                self.p.subs(
                    [
                        (self.theta1, theta_vals[0]),
                        (self.theta2, theta_vals[1]),
                        (self.theta3, theta_vals[2]),
                        (self.theta4, theta_vals[3]),
                        (self.theta5, theta_vals[4]),
                        (self.theta6, theta_vals[5]),
                    ]
                ),
                dtype=float,
            ).flatten()  # flatten to convert from 3x1 matrix to 1D array
            current_orientation = np.array(
                self.R.subs(
                    [
                        (self.theta1, theta_vals[0]),
                        (self.theta2, theta_vals[1]),
                        (self.theta3, theta_vals[2]),
                        (self.theta4, theta_vals[3]),
                        (self.theta5, theta_vals[4]),
                        (self.theta6, theta_vals[5]),
                    ]
                ),
                dtype=float,
            )
            # Position error
            position_error = target_pose - current_position
            R_error = target_orientation_matrix @ current_orientation.T

            theta_error = np.arccos(
                np.clip((np.trace(R_error) - 1) / 2, -1, 1)
            )  # angle error
            # clipping to avoid numerical issues outside the valid range of arccos
            # calculates the angle of rotation needed to align the current orientation with the target orientation

            # Store the position error
            position_errors.append(np.linalg.norm(position_error))
            orientation_errors.append(theta_error)

            if abs(theta_error) > rot_tolerance:
                # Orientation error (angle-axis representation) - avoid singularities
                u_error = (
                    1 / (2 * np.sin(theta_error))
                ) * np.array(  # u_error is the unit rotation axis for the orientation error
                    [
                        R_error[2][1] - R_error[1][2],
                        R_error[0][2] - R_error[2][0],
                        R_error[1][0] - R_error[0][1],
                    ]
                )

                if np.linalg.norm(u_error) > rot_tolerance:
                    u_error = u_error / np.linalg.norm(
                        u_error
                    )  # normalize to unit vector
                orientation_error = theta_error * u_error  # orientation error vector
            else:
                orientation_error = np.zeros(3)

            control_outputs = np.zeros(6)
            for i in range(3):
                control_outputs[i] = (
                    Kp_pos * position_error[i]
                )  # outputs [x, y, z, 0, 0, 0]
            for i in range(3):
                control_outputs[i + 3] = (
                    Kp_rot * orientation_error[i]
                )  # outputs [x, y, z, rx, ry, rz]

            J_val = np.array(
                self.J.subs(
                    [
                        (self.theta1, theta_vals[0]),
                        (self.theta2, theta_vals[1]),
                        (self.theta3, theta_vals[2]),
                        (self.theta4, theta_vals[3]),
                        (self.theta5, theta_vals[4]),
                        (self.theta6, theta_vals[5]),
                    ]
                ),
                dtype=float,
            )

            # Compute change in joint variables using pseudo-inverse of Jacobian
            delta_theta = (
                np.linalg.pinv(J_val, rcond=1e-4) @ control_outputs
            )  # TODO: this might fail bc different dimensions of J_val and control_outputs --> control_outputs used to be 3x1 instead of 6x1

            # Update joint variables
            theta_vals += delta_theta

            # self.log(
            #     f"Errors: {np.linalg.norm(position_error)} {np.linalg.norm(orientation_error)}"
            # )

            # Check convergence to the final desired position
            if (
                np.linalg.norm(position_error) < pos_tolerance
                and np.linalg.norm(orientation_error) < rot_tolerance
            ):
                break

        f_idx = 0
        while os.path.exists(os.path.join("data", f"hybrid_arm_{f_idx}.csv")):
            f_idx += 1

        if retry:
            f = open(os.path.join("data", f"hybrid_arm_{f_idx-1}_retry.csv"), "w")
        else:
            f = open(os.path.join("data", f"hybrid_arm_{f_idx}.csv"), "w")

        f.write("PositionError,OrientationError\n")
        for i in range(len(position_errors)):
            f.write(f"{position_errors[i]},{orientation_errors[i]}\n")
        f.close()

        if retry:
            return theta_vals.tolist()

        valid, theta_vals = self.check_pose(theta_vals, target_pose, target_orientation)
        self.log(f"{'Valid' if valid else 'Invalid'} pose: {theta_vals}")
        return theta_vals.tolist()

    def move(
        self,
        position: list[float],
        orientation: list[float],
        starting_joint_state: list[float],
    ):
        if position is None:
            return [0, 0, 0, 0, 0, 0]
        elif (
            len(position) != 3
            or len(orientation) != 3
            or len(starting_joint_state) != 6
        ):
            raise ValueError(
                "Position and orientation must be lists of 3 elements each, and starting_joint_state must be a list of 6 elements."
            )

        target_pose = np.array(position)
        target_orientation = np.array(orientation)
        starting_joint_state = np.array(starting_joint_state)

        self.log(
            f"Received request: {target_pose}, {target_orientation}, {starting_joint_state}"
        )

        return self.compute(target_pose, target_orientation, starting_joint_state)

    def plot(self, joints, target_position, target_orientation):
        vals = [
            np.array(
                self.T[i].subs(
                    [
                        (self.theta1, joints[0]),
                        (self.theta2, joints[1]),
                        (self.theta3, joints[2]),
                        (self.theta4, joints[3]),
                        (self.theta5, joints[4]),
                        (self.theta6, joints[5]),
                    ]
                ),
                dtype=float,
            )
            for i in range(6)
        ]

        fig = plt.figure()
        ax = fig.add_subplot(111, projection="3d")

        # Plot links with different colors
        ax.plot(
            [0, vals[0][0, 3]],
            [0, vals[0][1, 3]],
            [0, vals[0][2, 3]],
            "b-",
            linewidth=2,
        )
        ax.plot(
            [vals[0][0, 3], vals[1][0, 3]],
            [vals[0][1, 3], vals[1][1, 3]],
            [vals[0][2, 3], vals[1][2, 3]],
            "r-",
            linewidth=2,
        )
        ax.plot(
            [vals[1][0, 3], vals[2][0, 3]],
            [vals[1][1, 3], vals[2][1, 3]],
            [vals[1][2, 3], vals[2][2, 3]],
            "g-",
            linewidth=2,
        )
        ax.plot(
            [vals[2][0, 3], vals[3][0, 3]],
            [vals[2][1, 3], vals[3][1, 3]],
            [vals[2][2, 3], vals[3][2, 3]],
            "c-",
            linewidth=2,
        )
        ax.plot(
            [vals[3][0, 3], vals[4][0, 3]],
            [vals[3][1, 3], vals[4][1, 3]],
            [vals[3][2, 3], vals[4][2, 3]],
            "m-",
            linewidth=2,
        )
        ax.plot(
            [vals[4][0, 3], vals[5][0, 3]],
            [vals[4][1, 3], vals[5][1, 3]],
            [vals[4][2, 3], vals[5][2, 3]],
            "y-",
            linewidth=2,
        )

        ax.plot(
            target_position[0],
            target_position[1],
            target_position[2],
            "rx",
            markersize=12,
            linewidth=2,
        )

        target_orientation = Rotation.from_euler(
            "xyz", target_orientation, degrees=True
        ).as_matrix()
        v = np.dot(target_orientation, np.array([[0, 0, 1]]).T)
        ax.quiver(
            target_position[0],
            target_position[1],
            target_position[2],
            v[0],
            v[1],
            v[2],
            color="r",
            length=0.3,
        )

        ax.set_xlabel("X")
        ax.set_ylabel("Y")
        ax.set_zlabel("Z")  # type: ignore
        ax.set_xlim(-0.75, 0.75)
        ax.set_ylim(-0.75, 0.75)
        ax.set_zlim(0, 1.5)  # type: ignore
        plt.show()
