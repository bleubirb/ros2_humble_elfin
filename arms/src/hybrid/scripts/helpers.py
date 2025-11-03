from dataclasses import dataclass, field
import os

@dataclass
class DataBucket():
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
    classification: str = "unknown"  # fruit ripeness

    @staticmethod
    def header():
        return "Time,ReachedHoldTime,Force,DesiredForce,State,ActualWidth,CommandWidth,Proximity,RawFzL,RawFzR,ForcePrediction,SpringConstant,RLSError,Ripeness\n"

    def __str__(self):
        return f"{self.time},{self.hold_time},{self.force},{self.fd},{self.raw_fz_l},{self.raw_fz_r},{self.state},{self.width},{self.cmd_width},{self.prox},{self.F_pred},{self.k},{self.rls_error},{self.classification}\n"


@dataclass
class DataRecorder():
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
