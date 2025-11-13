#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import threading
import time
from statistics import mean

import numpy as np
import rclpy
from helpers import DataBucket, DataRecorder
from onrobot_rg2ft_control.OnRobotRG2FT import OnRobotRG2FT
from onrobot_rg2ft_msgs.msg import RG2FTCommand, RG2FTState

DISABLE_PNS_LOGGING = False

LAMBDA_FACTOR = 0.98
FORCE_SCALE = 1.0
CONTACT_REQUIRED = 3
MAX_ERROR_RATIO = 0.5  # reject updates with error > 50% of force
THRESHOLD_UNRIPE = 0.06
THRESHOLD_OVERRIPE = 0.015
MIN_BERRY_WIDTH = 50  # mm, min width to consider for berry
MAX_BERRY_WIDTH = 200  # mm, max width to consider for berry
MIN_WIDTH_EXIT_LOOPS = 5  # number of loops with width below min to exit force control


class PNS_Driver:
    def __init__(self, node, ip, port):
        self.node = node
        self.gripper = OnRobotRG2FT(ip, port)
        self.state_pub = node.create_publisher(RG2FTState, "state", 1)
        self.cmd_sub = node.create_subscription(
            RG2FTCommand, "command", self.gripper.writeCommand, 1
        )
        self.fd = 0
        self.done = True
        self.shutdown = False
        # self.P = 1000 * np.eye(2)
        self.P = 1000 * np.eye(1)
        # self.theta = np.zeros((2, 1))
        self.theta = np.zeros((1, 1))
        self.x0 = None
        self.initialized = False
        self.last_t = None
        self.last_x = None
        self.last_F = None

        self.rls_alpha = 0.5
        self.compression_ema = None
        self.force_ema = None
        self.last_compression_ema = None
        self.contact_counter = 0
        self.fruit_state = "unclassified"

        self.thread = threading.Thread(target=self.loop)
        self.thread.start()

        self.data = DataRecorder(self.node)

    def log(self, message):
        if not DISABLE_PNS_LOGGING:
            self.node.get_logger().info(message)

    def set_fd(self, force):
        self.fd = force
        self.done = False

    def get_done(self):
        return self.done

    def stop(self):
        self.shutdown = True
        self.thread.join()

    def update_rls(self, t, x, F):
        if not self.initialized:
            self.last_t = t
            self.last_x = x
            self.last_F = F

            self.initialized = True
            # return None, None, None, None
            return None, None, None

        dt = t - self.last_t
        if np.isclose(dt, 0, atol=1e-6) or dt < 0:
            # return None, None, None, None
            return None, None, None

        # v = (x - self.last_x) / dt

        compression = (self.x0 - x) if self.x0 is not None else -x
        # small-contact guard: if measured force is tiny, skip updating to avoid bias
        F_scaled = F / FORCE_SCALE
        CONTACT_THRESHOLD_N = 0.15
        if F_scaled < CONTACT_THRESHOLD_N:
            # not in contact, reset contact counter and skip update
            self.last_t = t
            self.last_x = x
            self.last_F = F
            # self.x0 = None
            self.contact_counter = 0
            # return None, None, None, None
            return None, None, None

        if self.x0 is None:
            if x < MIN_BERRY_WIDTH:
                self.log("Initial gripper width too small, berry is likely overripe.")
                return 0, None, None

            self.x0 = x
            self.log(f"Baseline width (x0) set to {self.x0:.2f}")

        # in contact, increment contact counter for contact_required consecutive samples, avoiding chatter responses
        self.contact_counter = min(CONTACT_REQUIRED, self.contact_counter + 1)

        # EMA for velocity
        if self.compression_ema is None:
            self.compression_ema = compression
            self.last_compression_ema = compression
        else:
            self.compression_ema = (
                self.rls_alpha * compression
                + (1 - self.rls_alpha) * self.compression_ema
            )

        if self.force_ema is None:
            self.force_ema = F_scaled
        else:
            self.force_ema = (
                self.rls_alpha * F_scaled + (1 - self.rls_alpha) * self.force_ema
            )

        if self.contact_counter < CONTACT_REQUIRED:
            # refresh last values to avoid large dt on next iteration
            self.last_t = t
            self.last_x = x
            self.last_F = F
            # return None, None, None, None
            return None, None, None

        # v_smooth = (
        #     (self.compression_ema - self.last_compression_ema) / dt
        # )
        # phi = np.array([[self.compression_ema], [v_smooth]])
        phi = np.array([[self.compression_ema]])

        F_pred = float(
            (self.theta.T @ phi).item()
        )  # F = k * compression + c * v_smooth
        error = self.force_ema - F_pred

        gain = (self.P @ phi) / (LAMBDA_FACTOR + (phi.T @ self.P @ phi))
        self.theta = self.theta + gain * error

        self.P = (self.P - gain @ phi.T @ self.P) / LAMBDA_FACTOR

        self.last_t = t
        self.last_x = x
        self.last_F = F

        if abs(error) > max(MAX_ERROR_RATIO * max(1e-6, F_scaled), 0.1):
            self.last_t = t
            self.last_x = x
            self.last_F = F
            self.last_compression_ema = self.compression_ema
            # return None, None, F_pred, error
            return None, F_pred, error

        # k, c = self.theta.flatten()
        # return k, c, F_pred, error
        k = self.theta.item()
        return k, F_pred, error

    def classify(self, k):
        if k is None:
            return "unknown"
        elif k > THRESHOLD_UNRIPE:
            return "unripe"
        elif k >= THRESHOLD_OVERRIPE and k <= THRESHOLD_UNRIPE:
            return "ripe"
        else:
            return "overripe"

    def loop(self):
        LOOP_FREQ = 100  # Hz
        PERIOD = 1.0 / LOOP_FREQ
        force_avg = 0
        # movement = 200 # for berry -> should not need this either, test without
        movement = 300  # for ball
        q = 0
        desired_force = 0
        last_prox = 1000

        MIN_HOLD_TIME = 10  # hold for 5 minutes -> testing temperature sensor drift
        MAX_GRIP_TIME = 300  # seconds -> adjust for altering data collection amount

        OPEN_WIDTH = 250  # mm; max opening width -> if smaller than 300 mm diameter, change open width to 400 mm

        # OPEN_WIDTH = 670  # only for water bottle experiment

        # states for gripper control
        MOVE = 4
        TIGHTEN_FAST = 3
        TIGHTEN = 2
        TIGHTEN_SLOW = 1
        HOLD = 0
        LOOSEN_SLOW = -1
        LOOSEN = -2

        # proximity redundancy -> should not need this anymore, test without
        FAR = 500
        CLOSE = 300

        MOVING_AVG_LEN_FORCE = 50
        MOVING_AVG_LEN_PROX = 100

        SLOW_FORCE_BOUND = 0.5

        # hysteresis thresholds
        DELTA2 = 0.3
        DELTA1 = 0.1

        # no hysteresis
        # DELTA2 = 0.0
        # DELTA1 = 0.0

        SPEED_FAST = 1.0
        SPEED_NORMAL = 0.5
        SPEED_SLOW = 0.1

        OPEN_HOLD_TOLERANCE = 5

        prox_l_range = []
        prox_r_range = []
        width_range = []
        force_range = []

        last_target = 0

        reached_hold_time = float("inf")
        start_time = float("inf")

        CALIBRATION_TIME = 5  # seconds

        calibrated = False
        l_force_bias = 0.0
        r_force_bias = 0.0
        l_force_drift = 0.0  # per loop increment
        r_force_drift = 0.0  # per loop increment
        # baseline_loop_counter = 0  # loop index at last calibration/rebaseline

        min_width_exit_counter = 0

        tmp_width = 0

        loop_counter = 0
        while rclpy.ok() and not self.shutdown:
            loop_start_time = time.time()
            state = self.gripper.readState()

            bucket = DataBucket(time=loop_start_time, cmd_width=last_target)

            if self.fd != desired_force:
                desired_force = self.fd
                reached_hold_time = float("inf")
                start_time = time.time()
                self.fruit_state = "unclassified"

            bucket.fd = desired_force

            cmd = RG2FTCommand()
            tmp_width = last_target  # mm
            cmd.target_force = movement  # target force is how aggressive the gripper moves, NOT the actual force applied
            cmd.control = 1

            prox_l_range.append(state.proximity_value_l)
            prox_r_range.append(state.proximity_value_r)
            width_range.append(state.actual_gripper_width)

            if len(prox_l_range) > MOVING_AVG_LEN_PROX:
                prox_l_range.pop(0)
                prox_r_range.pop(0)
                width_range.pop(0)

            ProxL = mean(prox_l_range)
            ProxR = mean(prox_r_range)
            width = mean(width_range)
            bucket.width = width

            # if cmd.target_width == 0:
            #     cmd.target_width = int(max(0, min(65535, round(width))))

            ProxAvg = (
                ProxL + ProxR
            ) / 2  # divide by two due to width between two fingers (find midpoint)
            bucket.prox = ProxAvg

            if (
                mean([state.proximity_value_l, state.proximity_value_r]) - last_prox
                > 200
            ):
                # desired_force = 0
                # self.fd = 0

                # attempt to regrip if object slipped, wait 5 seconds before regrip
                self.log("Object slip detected, attempting to regrip...")

                last_prox = ProxAvg

                tmp_width = OPEN_WIDTH
                last_target = tmp_width
                cmd.target_width = int(round(tmp_width))

                self.gripper.writeCommand(cmd)

                # mark move state in program state data
                q = MOVE
                reached_hold_time = float("inf")
                force_range = []

                delay_start_time = time.time()

                while time.time() - delay_start_time < 5:
                    tmp_bucket = DataBucket(
                        time=time.time(),
                        fd=desired_force,
                        state=q,
                        width=width,
                        cmd_width=last_target,
                        prox=ProxAvg,
                    )
                    self.data.record(tmp_bucket)

                    time.sleep(PERIOD)
                    loop_counter += 1

                # time.sleep(5)
                # desired_force = 1.5
                # self.fd = 1.5
                self.log("Regrip command sent, resuming force control...")
                q = TIGHTEN
                # self.done = False
                continue

            last_prox = ProxAvg

            # Raw force readings
            l_force_raw = abs(state.fz_l)
            r_force_raw = abs(state.fz_r)

            bucket.raw_fz_l = l_force_raw
            bucket.raw_fz_r = r_force_raw

            # if calibrated and self.enable_drift_comp:
            #     loops_since_baseline = max(0, loop_counter - baseline_loop_counter)
            #     l_force = l_force_raw - l_force_bias - l_force_drift * loops_since_baseline
            #     r_force = r_force_raw - r_force_bias - r_force_drift * loops_since_baseline
            # else:
            #     l_force = l_force_raw
            #     r_force = r_force_raw

            l_force = l_force_raw - l_force_bias - l_force_drift * loop_counter
            r_force = r_force_raw - r_force_bias - r_force_drift * loop_counter

            force = (l_force_raw + r_force_raw) / 2 / 10

            force_range.append(force)
            if len(force_range) > MOVING_AVG_LEN_FORCE:
                force_range.pop(0)
            force_avg = mean(force_range)
            bucket.force = force_avg

            force_error = desired_force - force_avg

            if desired_force == 0:
                tmp_width = OPEN_WIDTH
                q = HOLD
                if abs(width - tmp_width) < OPEN_HOLD_TOLERANCE:
                    reached_hold_time = min(reached_hold_time, time.time())
            else:
                if ProxAvg > FAR and (
                    force_avg <= (desired_force - desired_force * SLOW_FORCE_BOUND)
                ):
                    q = TIGHTEN_FAST
                elif (
                    ProxAvg < FAR
                    and ProxAvg > CLOSE
                    and (
                        force_avg <= (desired_force - desired_force * SLOW_FORCE_BOUND)
                    )
                ):
                    q = TIGHTEN
                else:
                    # record current width
                    # diameter_approx = state.actual_gripper_width
                    if (q == TIGHTEN or q == TIGHTEN_SLOW or q == TIGHTEN_FAST) and (
                        force_error <= (desired_force * DELTA1)
                    ):
                        reached_hold_time = time.time()
                        q = HOLD
                    elif (q == HOLD) and (force_error >= (desired_force * DELTA2)):
                        q = TIGHTEN_SLOW
                    elif (q == HOLD) and (force_error <= (desired_force * -1 * DELTA2)):
                        q = LOOSEN_SLOW
                    elif (q == LOOSEN or q == LOOSEN_SLOW) and (
                        force_error >= (desired_force * -1 * DELTA1)
                    ):
                        reached_hold_time = time.time()
                        q = HOLD

                    # for parameter estimation of spring constant (remove if no estimation)
                    if width < MAX_BERRY_WIDTH:
                        # [k, _, F_pred, error] = self.update_rls(time.time(), width, force_avg)
                        [k, F_pred, error] = self.update_rls(
                            time.time(), width, force_avg
                        )
                        self.fruit_state = self.classify(k)
                        bucket.k = k
                        bucket.F_pred = F_pred
                        bucket.rls_error = error
                        bucket.baseline_w = self.x0 if self.x0 is not None else 0.0
                        bucket.classification = self.fruit_state
                        if k is None:
                            self.log("Estimated spring constant: unknown")
                        else:
                            self.log(
                                f"Estimated spring constant: {k:.5f} N/mm, fruit state: {self.fruit_state}, F_pred: {F_pred:.2f}, F: {force_avg:.2f}, baseline width (x0): {self.x0:.2f} mm"
                            )

                    if width < MIN_BERRY_WIDTH:
                        min_width_exit_counter += 1
                        self.log(
                            f"Width below minimum threshold ({MIN_BERRY_WIDTH} mm) for {min_width_exit_counter} loops."
                        )
                    else:
                        min_width_exit_counter = 0

                bucket.state = q

                if q != HOLD:
                    reached_hold_time = float("inf")

                if q == HOLD:
                    pass
                elif q == TIGHTEN:  # move smaller
                    tmp_width -= SPEED_NORMAL
                elif q == LOOSEN:
                    tmp_width += SPEED_NORMAL
                elif q == TIGHTEN_FAST:
                    tmp_width -= SPEED_FAST
                elif q == LOOSEN_SLOW:
                    tmp_width += SPEED_SLOW
                elif q == TIGHTEN_SLOW:
                    tmp_width -= SPEED_SLOW

                if tmp_width < 0:
                    tmp_width = 0
                elif tmp_width > 1000:
                    tmp_width = 1000

                self.log(
                    f"prox: {ProxAvg:.2f}, target width: {tmp_width:.2f}, current force: {force_avg:.2f}, force: {force:.2f}, q state: {q}, raw fz_l: {l_force_raw:.2f}, raw fz_r: {r_force_raw:.2f}"
                )

            bucket.hold_time = (
                reached_hold_time if reached_hold_time != float("inf") else 0
            )

            if calibrated:
                self.data.record(bucket)

            cmd.target_width = int(round(tmp_width))
            # cmd.target_force = int(round(cmd.target_force))

            if (desired_force == 0 and not self.done) or cmd.target_width != int(
                round(last_target)
            ):
                self.gripper.writeCommand(cmd)
                if cmd.target_width == OPEN_WIDTH and not calibrated:
                    time.sleep(1)
                    self.log("Starting gripper force calibration...")
                    f_l_arr = []
                    f_r_arr = []
                    for _ in range(MOVING_AVG_LEN_FORCE):
                        state = self.gripper.readState()
                        f_l_arr.append(abs(state.fz_l))
                        f_r_arr.append(abs(state.fz_r))
                        time.sleep(PERIOD)

                    l_force_bias = mean(f_l_arr)
                    r_force_bias = mean(f_r_arr)

                    time.sleep(CALIBRATION_TIME)

                    f_l_arr = []
                    f_r_arr = []
                    for _ in range(MOVING_AVG_LEN_FORCE):
                        state = self.gripper.readState()
                        f_l_arr.append(abs(state.fz_l))
                        f_r_arr.append(abs(state.fz_r))
                        time.sleep(PERIOD)

                    l_force_drift = (mean(f_l_arr) - l_force_bias) / (
                        (CALIBRATION_TIME + PERIOD * MOVING_AVG_LEN_FORCE) * LOOP_FREQ
                    )
                    r_force_drift = (mean(f_r_arr) - r_force_bias) / (
                        (CALIBRATION_TIME + PERIOD * MOVING_AVG_LEN_FORCE) * LOOP_FREQ
                    )
                    # baseline_loop_counter = loop_counter
                    self.log(
                        f"Force calibration complete! l_force_drift: {l_force_drift:.6f} per loop, r_force_drift: {r_force_drift:.6f} per loop\n Bias left: {l_force_bias:.6f}, Bias right: {r_force_bias:.6f}\n"
                    )
                    calibrated = True
                    # set baseline open width for compression calculation

            # if self.enable_drift_comp and self.enable_online_rebaselining:
            #     no_contact = desired_force == 0 and (ProxAvg > FAR or cmd.target_width >= OPEN_WIDTH - OPEN_HOLD_TOLERANCE)
            #     if no_contact and calibrated:
            #         alpha = max(0.0, min(1.0, self.rebaselining_alpha))
            #         l_force_bias = (1 - alpha) * l_force_bias + alpha * l_force_raw
            #         r_force_bias = (1 - alpha) * r_force_bias + alpha * r_force_raw
            #         baseline_loop_counter = loop_counter

            last_target = tmp_width

            if (
                not self.done
                and desired_force == self.fd
                and (
                    time.time() - reached_hold_time > MIN_HOLD_TIME
                    or time.time() - start_time > MAX_GRIP_TIME
                    or desired_force == 0
                )
            ) or (min_width_exit_counter >= MIN_WIDTH_EXIT_LOOPS):
                self.done = True
                if self.fruit_state == "overripe":
                    self.log("overripe berry detected, releasing grip")
                    self.fd = 0
                    desired_force = 0
                    tmp_width = OPEN_WIDTH
                    cmd.target_width = int(round(tmp_width))
                    self.gripper.writeCommand(cmd)
                    last_prox = 1000
                elif self.fruit_state == "unknown":
                    self.log("unable to estimate berry ripeness, releasing grip")
                    self.fd = 0
                    desired_force = 0
                    tmp_width = OPEN_WIDTH
                    cmd.target_width = int(round(tmp_width))
                    self.gripper.writeCommand(cmd)
                    last_prox = 1000
                elif self.fruit_state == "ripe":
                    self.log("ripe berry detected, holding grip")
            # else:
            #     self.log(f"\nTime left: {MAX_GRIP_TIME - (time.time() - start_time):.2f}\n")

            duration = time.time() - loop_start_time
            if duration < PERIOD:
                time.sleep(PERIOD - duration)

            loop_counter += 1

        self.data.save()
