import os
from concurrent.futures import Future, ThreadPoolExecutor
from dataclasses import dataclass, field
from enum import Enum

from cmd_move import CmdMove
from move_solver import MoveSolver
from rclpy.node import Node


@dataclass
class DataBucket:
    time: float
    hold_time: float = 0.0
    force: float = 0.0
    fd: float = 0.0
    raw_fz_l: float = 0.0
    raw_fz_r: float = 0.0
    state: int = 0
    width: float = 0.0
    cmd_width: float = 0.0
    prox: float = 0.0
    F_pred: float = 0.0
    k: float = 0.0
    rls_error: float = 0.0
    baseline_w: float = 0.0
    classification: str = "unknown"  # fruit ripeness

    @staticmethod
    def header():
        return "Time,ReachedHoldTime,Force,DesiredForce,State,ActualWidth,CommandWidth,Proximity,RawFzL,RawFzR,ForcePrediction,SpringConstant,RLSError,BaselineWidth,Ripeness\n"

    def __str__(self):
        return f"{self.time},{self.hold_time},{self.force},{self.fd},{self.raw_fz_l},{self.raw_fz_r},{self.state},{self.width},{self.cmd_width},{self.prox},{self.F_pred},{self.k},{self.rls_error},{self.baseline_w},{self.classification}\n"


@dataclass
class DataRecorder:
    node: Node
    data: list[DataBucket] = field(default_factory=list)

    def record(self, bucket: DataBucket):
        self.data.append(bucket)

    def save(self):
        if not os.path.exists("data"):
            os.makedirs("data")
        f_idx = 0
        while os.path.exists(f"data/hybrid_gripper_{f_idx}.csv"):
            f_idx += 1
        with open(f"data/hybrid_gripper_{f_idx}.csv", "w") as f:
            f.write(DataBucket.header())
            for bucket in self.data:
                f.write(str(bucket))

        self.node.get_logger().info(f"Data saved to data/hybrid_gripper_{f_idx}.csv")


class Action(Enum):
    MOVE = 0
    GRIP = 1
    FIND = 2


@dataclass
class JointAction:
    action: Action

    # if action == MOVE
    position: list[float] | None = None
    orientation: list[float] | None = None

    # if action == GRIP
    force: float | None = None

    ms: MoveSolver | None = None
    cm: CmdMove | None = None

    executor: ThreadPoolExecutor | None = None
    future: Future | None = None

    letter: str | None = None
    raised: bool | None = None
    joints: list[float] | None = None

    def __post_init__(self):
        if self.action == Action.MOVE:
            if (self.position is None or self.orientation is None) and not (
                self.position is None and self.orientation is None
            ):
                raise ValueError(
                    "Position and orientation must both be None or list[float] for MOVE action."
                )

            if self.ms is None or self.cm is None:
                return

            self.future = self.executor.submit(
                self.ms.move,
                self.position,
                self.orientation,
                self.cm.joint_state or [0, 0, 0, 0, 0, 0],
            )

        elif self.action == Action.GRIP:
            if self.force is None:
                raise ValueError("Force must be provided for GRIP action.")
