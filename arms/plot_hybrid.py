import os.path

from matplotlib import pyplot as plt
import numpy as np

# gripper, state, arm_first, arm_end (inclusive), desired force, hysteresis
data = [
    # ("Position A", 4, 7, 30, 35, True), # pos = (0, 0.600, 0.450), rot = (25.6, -49, -122.45)
    # ("Position A", 5, 8, 36, 41, True), 
    # ("Position A", 6, 9, 42, 47, True), 
    # ("Position B", 19, 23, 72, 77, True), # pos = (0.200, 0.650, 0.295), rot = (25.6, -49, -112.45) - pos w/o end effector
    # ("Position B", 20, 24, 78, 83, True), 
    # ("Position B", 21, 25, 84, 89, True), 
    # ("Position B", 22, 26, 90, 95, True), 
    # ("Position B", 23, 27, 96, 101, True), 
    # ("Position C", 31, 35, 111, 116, True), # pos = (0.040, 0.520, 0.4), rot = (-90, -112, 0)
    # ("Position C", 32, 36, 117, 122, True), 
    # ("Position C", 33, 38, 125, 130, True), 
    # ("No Hysteresis", 38, 44, 152, 157, False), # pos = (-0.300, 0.550, 0.360), rot = (-90, -22, 0)
    # ("No Hysteresis", 40, 49, 169, 174, False), 
    # ("No Hysteresis", 41, 50, 175, 180, False), 
    # ("No Hysteresis", 43, 52, 187, 192, False), 
    # ("No Hysteresis", 44, 53, 193, 198, False), 
    # ("Stress Ball, 5N", 59, 68, 213, 218, True), # pos = (0.040, 0.620, 0.4), rot = (-90, -112, 0)
    # ("Stress Ball, 5N", 60, 69, 219, 224, True), 
    # ("Stress Ball, 7N", 61, 70, 225, 230, True), 
    # ("Stress Ball, 3N", 62, 71, 231, 236, True), 
    # ("Stress Ball, 3N, No Hysteresis", 63, 72, 237, 242, False), 
    # ("Stress Ball, 5N, No Hysteresis", 64, 73, 243, 248, False), 
    # ("Stress Ball, 7N, No Hysteresis", 65, 74, 249, 254, False), 
    # ("Stress Ball, Dropped", 71, 81, 286, 291, True), 
# pre demo:
    # ("Test 0", 1025, 1032, None, None, True),
    # ("Test 1", 1028, 1035, None, None, True),
    # ("Test 2", 1029, 1036, None, None, True),
    # ("Test 3", 1030, 1037, None, None, True),
    # ("Test 4", 1032, 1039, None, None, True),
    # ("Test 5", 1039, 1046, None, None, True),
    # ("Test 6", 1049, 1061, None, None, True),
    # ("Test 7", 1004, 1006, None, None, True),
    # ("Test 8", 1007, 1009, None, None, True),
    # ("Test 9", 1008, 1010, None, None, True),
# demo (12/05/2024)
    # ("Live Demo", 1009, 1011, None, None, True),
    # ("Test 10", 1010, 1012, None, None, True),
    # miss one
    # ("Test 11", 1013, 1015, None, None, True),
    # miss two
    # ("Test 12", 1016, 1018, None, None, True),
    # ("Test 13", 1017, 1019, None, None, True),
    # ("Test 14", 1018, 1020, None, None, True),
    # ("Test 15", 1019, 1021, None, None, True),
    # miss one
    # ("Test 16", 1021, 1023, None, None, True),
# stress ball 12/11/2024
    # ("Ball 1", 1027, 1029, None, None, True),
    # ("Ball 2", 1034, 1033, None, None, True),
# simulations
    # ("Simulation", 0, None, 500, 500, True),
    # ("Simulation", 1, None, None, None, True),
    # ("Ball", 74, 85, None, None, True),
    # ("Ball", 75, 86, None, None, True),
    # ("Ball", 77, 88, None, None, True),
    # ("Ball", 79, 90, None, None, True),
    # ("Ball", 80, 91, None, None, True),
    # ("Ball", 82, 93, None, None, True),
    # ("Ball", 83, 94, None, None, True),
    # ("Ball", 84, 95, None, None, True),
    # ("Ball", 86, 98, None, None, True),
    # ("Ball", 89, 101, None, None, True), # THIS IS THE GOOD ONE (5N ball)
    # ("Ball", 90, 102, None, None, True), # This is a hard maybe, but so close
    # ("Ball", 92, 104, None, None, True), # if the force was 4, it would be perfect (4 -> 3N)

# simulations 7/10/2025
    # ("Ball", 80, 100, None, None, True),
    # ("Ball", 84, 104, None, None, True),
    # ("Ball", 85, 105, None, None, True),
    # ("Ball", 92, 113, None, None, True),
    # ("Ball", 93, 114, None, None, True),
    # ("Ball", 94, 115, None, None, True),
    # ("Ball", 95, 116, None, None, True),
    # ("Ball", 96, 117, None, None, True),
    # ("Ball", 99, 120, None, None, True),
    # ("Ball", 100, 121, None, None, True),
    # ("Ball", 101, 122, None, None, True),
    # ("Ball", 102, 123, None, None, True),
    # ("Water Bottle", 111, 132, None, None, True),
    # ("Water Bottle", 123, 145, None, None, True),
    # ("Water Bottle", 124, 146, None, None, True),
    # ("Water Bottle", 125, 147, None, None, True),
    # ("Water Bottle", 127, 149, None, None, True),
    # ("Water Bottle", 130, 152, None, None, True),
    # ("Water Bottle", 131, 153, None, None, True),
    # ("Water Bottle", 132, 154, None, None, True),
    ("Water Bottle Rotation", 133, 155, None, None, True), # rotation
    ("Water Bottle Rotation", 134, 156, None, None, True), # rotation
    # ("Water Bottle", 135, 157, None, None, True),
    ("Water Bottle Rotation", 136, 158, None, None, True), # rotation
    ("Water Bottle Rotation", 137, 159, None, None, True), # rotation
    ("Water Bottle Rotation", 138, 161, None, None, True), # rotation
    ("Water Bottle Squeeze", 141, 164, None, None, True), # squeeze
    # ("Water Bottle Squeeze", 143, 166, None, None, True), # squeeze no good
    ("Water Bottle Squeeze", 145, 168, None, None, True), # squeeze
    # TODO: water bottle fill
    ("Water Bottle Fill", 146, 169, None, None, True), # fill
    ("Water Bottle Fill", 147, 170, None, None, True), # fill
    ("Water Bottle Fill", 148, 171, None, None, True), # fill
    ("Water Bottle Fill", 149, 172, None, None, True), # fill
    ("Water Bottle Fill", 150, 173, None, None, True), # fill
]

TOLERANCE = 0.05
DELTA2 = 0.3
DELTA1 = 0.1

if not os.path.exists("plots"):
    os.makedirs("plots")
elif not os.path.isdir("plots"):
    print("plots is not a directory?")
    exit(1)

files = set(os.listdir("plots"))

for idx, item in enumerate(data):
    title, gripper, state, arm_first, arm_end, hyst = item
    print(title)
    start_time = -1

    g_file = None
    if gripper is None:
        print(f"Skipping gripper for {title}")
    elif os.path.exists(f"../arms/data/hybrid_gripper_{gripper}.csv"):
        g_file = open(f"../arms/data/hybrid_gripper_{gripper}.csv", "r").readlines()
        # start_time = min(start_time, float(g_file[1].split(",")[0]))
        start_time = max(start_time, float(g_file[1].split(",")[0]))
    else:
        print(f"Couldn't find arms/data/hybrid_gripper_{gripper}.csv")

    s_file = None
    if state is None:
        print(f"Skipping state for {title}")
    elif os.path.exists(f"../arms/data/hybrid_state_{state}.csv"):
        s_file = open(f"../arms/data/hybrid_state_{state}.csv", "r").readlines()
        # start_time = min(start_time, float(s_file[1].split(",")[0]))
        start_time = max(start_time, float(s_file[1].split(",")[0]))
    else:
        print(f"Couldn't find arms/data/hybrid_state_{state}.csv")

    # gripper - Time,ReachedHoldTime,Force,DesiredForce,State,ActualWidth,CommandWidth,Proximity
    # state - Time,State,X,Y,Z,RX,RY,RZ,F

    # program state over time
    time_1 = [0.0]
    program_state_1 = [0]
    boxy_1 = True
    if s_file:
        for line in s_file[1:]:
            line = line.split(",")
            if float(line[0]) < start_time:
                continue

            t = float(line[0]) - start_time
            if boxy_1:
                time_1.append(t)
                program_state_1.append(program_state_1[-1])
            time_1.append(t)
            program_state_1.append(int(line[1]))

        plt.figure()
        plt.plot(time_1, program_state_1)
        # plt.title(f"{title} - Program State")
        plt.xlabel("Time (s)")
        plt.ylabel("State")
        plt.yticks(range(0, 1 + 1), ["MOVE", "GRIP"])
        plt.savefig(f"plots/{idx}_{title}_program_state.png")
        # plt.show()
        plt.close()
        if f"{idx}_{title}_program_state.png" in files:
            files.remove(f"{idx}_{title}_program_state.png")

    # gripper state over time
    time_2 = [0.0]
    gripper_state_2 = [0]
    boxy_2 = False
    simple_states_2 = True
    if g_file:
        for line in g_file[1:]:
            line = line.split(",")
            if float(line[0]) < start_time:
                continue

            t = float(line[0]) - start_time
            if boxy_2:
                time_2.append(t)
                gripper_state_2.append(gripper_state_2[-1])
            time_2.append(t)
            if simple_states_2:
                if int(line[4]) == 0:
                    gripper_state_2.append(0)
                elif int(line[4]) > 0:
                    gripper_state_2.append(1)
                else:
                    gripper_state_2.append(-1)
            else:
                gripper_state_2.append(int(line[4]))

        plt.figure()
        plt.plot(time_2, gripper_state_2)
        # plt.title(f"{title} - Gripper State")
        plt.xlabel("Time (s)")
        plt.ylabel("State")
        if simple_states_2:
            plt.yticks(
                range(-1, 1 + 1),
                ["LOOSEN", "HOLD", "TIGHTEN"],
            )
        else:
            plt.yticks(
                range(-2, 3 + 1),
                ["LOOSEN", "LOOSEN\nSLOW", "HOLD", "TIGHTEN\nSLOW", "TIGHTEN", "TIGHTEN\nFAST"],
            )
        plt.savefig(f"plots/{idx}_{title}_gripper_state.png")
        # plt.show()
        plt.close()
        if f"{idx}_{title}_gripper_state.png" in files:
            files.remove(f"{idx}_{title}_gripper_state.png")

    # gripper force over time
    time_3 = []
    force_3 = []
    fd_3 = []

    # remove_start = 0
    # remove_end = 0

    if g_file:
        for i, line in enumerate(g_file[1:]):
            line = line.split(",")
            if float(line[0]) < start_time:
                continue

            t = float(line[0]) - start_time
            time_3.append(t)
            force_3.append(float(line[2]))
            if (fd_3 and abs(float(line[3]) - fd_3[-1]) > 0) or not float(line[3]):
                fd_3.append(np.nan)
            else:
                fd_3.append(float(line[3]))

        plt.figure()
        plt.plot(time_3, force_3)
        if hyst:
            fd2_top = [np.nan if a == np.nan else a * (1 + TOLERANCE) + DELTA2 for a in fd_3]
            fd2_bot = [np.nan if a == np.nan else a * (1 - TOLERANCE) - DELTA2 for a in fd_3]

            fd1_top = [np.nan if a == np.nan else a * (1 + TOLERANCE) + DELTA1 for a in fd_3]
            fd1_bot = [np.nan if a == np.nan else a * (1 - TOLERANCE) - DELTA1 for a in fd_3]
            
            plt.plot(
                time_3,
                fd2_bot,
                "--",
                color="red",
                label="Hysteresis Outer Bound (Lower)",
            )
            plt.plot(
                time_3,
                fd2_top,
                "--",
                color="red",
                label="Hysteresis Outer Bound (Upper)",
            )
            
            plt.plot(
                time_3,
                fd1_bot,
                "--",
                color="green",
                label="Hysteresis Inner Bound (Lower)",
            )
            plt.plot(
                time_3,
                fd1_top,
                "--",
                color="green",
                label="Hysteresis Inner Bound (Upper)",
            )
        else:
            plt.plot(
                time_3,
                fd_3,
                "--",
                color="red",
                label="Desired Force",
            )

        # plt.title(f"{title} - Force")
        plt.xlabel("Time (s)")
        plt.ylabel("Force (N)")
        plt.ylim(0, 4)
        plt.savefig(f"plots/{idx}_{title}_force.png")
        # plt.show()
        plt.close()
        if f"{idx}_{title}_force.png" in files:
            files.remove(f"{idx}_{title}_force.png")

    # gripper state over time
    time_4 = []
    cmd_width_4 = []
    if g_file:
        for line in g_file[1:]:
            line = line.split(",")
            if float(line[0]) < start_time:
                continue

            t = float(line[0]) - start_time
            time_4.append(t)
            cmd_width_4.append(float(line[6]))

        plt.figure()
        plt.plot(time_4, cmd_width_4)
        # plt.title(f"{title} - Command Width")
        plt.xlabel("Time (s)")
        plt.ylabel("Width (mm)")
        plt.savefig(f"plots/{idx}_{title}_cmd_width.png")
        # plt.show()
        plt.close()
        if f"{idx}_{title}_cmd_width.png" in files:
            files.remove(f"{idx}_{title}_cmd_width.png")

    # continue

    # 6 arm plots
    # 1. home to 10cm from target
    # 2. 10cm from target to target
    # 3. target to 10cm from target
    # 4. 10cm from target to 10cm from basket
    # 5. 10cm from basket to basket
    # 6. basket to 10cm from basket

    pos_errors = [0.0] * 100
    ori_errors = [0.0] * 100
    arm_exists = False
    if arm_first is not None and arm_end is not None:
        for arm in range(arm_first, arm_end + 1):
            a_file = None
            if os.path.exists(f"arms/data/hybrid_arm_{arm}.csv"):
                a_file = open(f"arms/data/hybrid_arm_{arm}.csv", "r").readlines()
            else:
                print(f"Couldn't find arms/data/hybrid_arm_{arm}.csv")

            if a_file:
                arm_exists = True
                for i, line in enumerate(a_file[1:]):
                    line = line.split(",")
                    if float(line[0]) < start_time:
                        continue

                    pos_error = float(line[0])
                    ori_error = float(line[1])
                    pos_errors[i] += pos_error
                    ori_errors[i] += ori_error
    if arm_exists:
        plt.figure()
        plt.plot(range(1, 101), [x / (arm_end - arm_first + 1) for x in pos_errors], label="Position Error")
        plt.plot(range(1, 101), [x / (arm_end - arm_first + 1) for x in ori_errors], label="Orientation Error")
        # plt.title(f"{title} - Average Error Over Time")
        plt.xlabel("Time (s)")
        plt.ylabel("Error (m/rad)")
        plt.legend()
        plt.savefig(f"plots/{idx}_{title}_arm_error.png")
        # plt.show()
        plt.close()
    
        if f"{idx}_{title}_arm_error.png" in files:
            files.remove(f"{idx}_{title}_arm_error.png")

        # if os.path.exists(f"data/hybrid_arm_{arm}_retry.csv"):
        #     print(f"Found data/hybrid_arm_{arm}_retry.csv")
        #     arms.append(open(f"data/hybrid_arm_{arm}_retry.csv", "r").readlines())

# count = 0
# pos_errors = [0.0] * 100
# rot_errors = [0.0] * 100
# for _, _, arm_start, arm_end, _, _ in data.values():
#     for arm in range(arm_start, arm_end + 1):
#         if os.path.exists(f"data/hybrid_arm_{arm}.csv"):
#             arm_file = open(f"data/hybrid_arm_{arm}.csv", "r").readlines()
#             for i, line in enumerate(arm_file[1:]):
#                 line = line.split(",")
#                 pos_error = float(line[0])
#                 rot_error = float(line[1])
#                 pos_errors[i] += pos_error
#                 rot_errors[i] += rot_error

#                 count += 1

#         # if os.path.exists(f"data/hybrid_arm_{arm}_retry.csv"):
#         #     arm_file = open(f"data/hybrid_arm_{arm}_retry.csv", "r").readlines()
#         #     for i, line in enumerate(arm_file[1:]):
#         #         line = line.split(",")
#         #         pos_error = float(line[0])
#         #         rot_error = float(line[1])
#         #         pos_errors[i] += pos_error
#         #         rot_errors[i] += rot_error

# pos_errors = [x / count for x in pos_errors]
# rot_errors = [x / count for x in rot_errors]
# # print(pos_errors)
# # print(rot_errors)

# plt.figure()
# plt.plot(range(1, 101), pos_errors, label="Position Error")
# plt.plot(range(1, 101), rot_errors, label="Orientation Error")
# plt.title("Average Error Over Time")
# plt.xlabel("Time (s)")
# plt.ylabel("Error (m/rad)")
# plt.legend()
# plt.savefig("plots/average_error.png")
# # plt.show()
# plt.close()

for f in files:
    try:
        os.remove(f"plots/{f}")
        print(f"removing {f}")
    except Exception as e:
        print(type(e), e)