import os
from datalog import *
import mmap
from multiprocessing import Pool
import tqdm
import json
import math

FOLDER = ""
WHEEL_RADIUS = 2 * 0.0254
MAX_WHEEL_DELTA = 10


def array_sum(array):
    result = 0
    for num in array:
        result += num
    return result


def get_stats(filename):
    with open(os.path.join(FOLDER, filename), "r") as f:
        try:
            mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
        except:
            print("Failed to parse", filename)
            return None

        reader = DataLogReader(mm)
        if not reader:
            print("Failed to parse", filename)
            return None

        # Read the log!
        key_ids = {
            "/Timestamp": -1,
            "/DriverStation/Enabled": -1,
            "/DriverStation/FMSAttached": -1,
            "/DriverStation/Autonomous": -1,
            "/RealOutputs/CommandsAll/DriveWithJoysticks": -1,
            "/PowerDistribution/ChannelCurrent": -1,
            "/SystemStats/BatteryVoltage": -1,
            "/SystemStats/BatteryCurrent": -1,
            "/Drive/Module0/DriveCurrentAmps": -1,
            "/Drive/Module1/DriveCurrentAmps": -1,
            "/Drive/Module2/DriveCurrentAmps": -1,
            "/Drive/Module3/DriveCurrentAmps": -1,
            "/Drive/Module0/TurnCurrentAmps": -1,
            "/Drive/Module1/TurnCurrentAmps": -1,
            "/Drive/Module2/TurnCurrentAmps": -1,
            "/Drive/Module3/TurnCurrentAmps": -1,
            "/Arm/ShoulderCurrentAmps": -1,
            "/Arm/ElbowCurrentAmps": -1,
            "/Arm/WristCurrentAmps": -1,
            "/Gripper/CurrentAmps": -1,
            "/CubeIntake/ArmCurrentAmps": -1,
            "/CubeIntake/RollerCurrentAmps": -1,
            "/Drive/Module0/DrivePositionRad": -1,
            "/Drive/Module1/DrivePositionRad": -1,
            "/Drive/Module2/DrivePositionRad": -1,
            "/Drive/Module3/DrivePositionRad": -1,
            "/AprilTagVision/Inst0/FrameCount": -1,
            "/AprilTagVision/Inst1/FrameCount": -1,
            "/AprilTagVision/Inst2/FrameCount": -1,
            "/AprilTagVision/Inst3/FrameCount": -1,
            "/ArmSolver/ParameterHash": -1,
            "/Arm/ShoulderRelativePositionRad": -1,
            "/Arm/ElbowRelativePositionRad": -1,
            "/Arm/WristRelativePositionRad": -1,
            "/CubeIntake/ArmRelativePositionRad": -1
        }
        stats = {
            "loop_cycles": 0,
            "time_total": 0.0,
            "time_teleop": 0.0,
            "time_auto": 0.0,
            "total_power": 0.0,
            "subsystem_power": {
                "control_system": 0.0,
                "drive": 0.0,
                "turn": 0.0,
                "shoulder": 0.0,
                "elbow": 0.0,
                "wrist": 0.0,
                "gripper": 0.0,
                "intake_arm": 0.0,
                "intake_roller": 0.0
            },
            "distance_meters": [0.0, 0.0, 0.0, 0.0],
            "distance_meters_auto": [0.0, 0.0, 0.0, 0.0],
            "vision_frames": 0,
            "arm_trajectories": 0,
            "shoulder_rotations": 0,
            "elbow_rotations": 0,
            "wrist_rotations": 0,
            "intake_rotations": 0,
            "is_match": False,
            "filename": filename
        }
        last_time = 0.0
        loop_cycle_time = 0.0
        enabled = False
        auto_mode = False
        joystick_drive = False
        fms_attached = False
        last_drive_position = [None, None, None, None]
        vision_frame_counts = [0, 0, 0, 0]
        last_shoulder_position = 0
        last_elbow_position = 0
        last_wrist_position = 0
        last_intake_position = 0
        subsystem_current_draws = {
            "mini_power_module": -1,
            "rio": -1,
            "radio": -1,
            "drive": [-1, -1, -1, -1],
            "turn": [-1, -1, -1, -1],
            "shoulder": -1,
            "elbow": -1,
            "wrist": -1,
            "gripper": -1,
            "intake_arm": -1,
            "intake_roller": -1
        }
        voltage = -1
        for record in reader:
            if record.isStart():
                start_data = record.getStartData()
                if start_data.name in key_ids:
                    key_ids[start_data.name] = start_data.entry

            else:
                if record.entry == key_ids["/Timestamp"]:
                    # Update loop cycle count
                    stats["loop_cycles"] += 1

                    # Add power usage from previous cycle
                    if voltage != -1:
                        if subsystem_current_draws["mini_power_module"] != -1 and subsystem_current_draws["rio"] != -1 and subsystem_current_draws["radio"] != -1:
                            stats["subsystem_power"]["control_system"] += (voltage *
                                                                           (subsystem_current_draws["mini_power_module"] + subsystem_current_draws["rio"] + subsystem_current_draws["radio"]) * loop_cycle_time) / 3600.0
                        if -1 not in subsystem_current_draws["drive"]:
                            stats["subsystem_power"]["drive"] += (voltage *
                                                                  array_sum(subsystem_current_draws["drive"]) * loop_cycle_time) / 3600.0
                        if -1 not in subsystem_current_draws["turn"]:
                            stats["subsystem_power"]["turn"] += (voltage *
                                                                 array_sum(subsystem_current_draws["turn"]) * loop_cycle_time) / 3600.0
                        for name in ["shoulder", "elbow", "wrist", "gripper", "intake_arm", "intake_roller"]:
                            if subsystem_current_draws[name] != -1:
                                stats["subsystem_power"][name] += (voltage *
                                                                   subsystem_current_draws[name] * loop_cycle_time) / 3600.0

                    # Add vision frame data
                    for i in range(4):
                        stats["vision_frames"] += vision_frame_counts[i]

                    # Add time delta
                    current_time = record.getInteger() / 1000000
                    loop_cycle_time = current_time - last_time
                    stats["time_total"] += loop_cycle_time
                    if enabled:
                        if auto_mode or not joystick_drive:
                            stats["time_auto"] += loop_cycle_time
                        else:
                            stats["time_teleop"] += loop_cycle_time
                    last_time = current_time

                    # Mark as match if enabled in auto while connected to FMS
                    if fms_attached and auto_mode and enabled:
                        stats["is_match"] = True

                elif record.entry == key_ids["/DriverStation/Enabled"]:
                    enabled = record.getBoolean()
                elif record.entry == key_ids["/DriverStation/FMSAttached"]:
                    fms_attached = record.getBoolean()
                elif record.entry == key_ids["/DriverStation/Autonomous"]:
                    auto_mode = record.getBoolean()
                elif record.entry == key_ids["/RealOutputs/CommandsAll/DriveWithJoysticks"]:
                    joystick_drive = record.getBoolean()
                elif record.entry == key_ids["/SystemStats/BatteryVoltage"]:
                    voltage = record.getDouble()
                elif record.entry == key_ids["/PowerDistribution/ChannelCurrent"]:
                    subsystem_current_draws["mini_power_module"] = record.getDoubleArray()[9]
                elif record.entry == key_ids["/SystemStats/BatteryCurrent"]:
                    rio_current = record.getDouble()
                    subsystem_current_draws["rio"] = rio_current
                    # Radio current not logged properly by akit through PDH, estimate based on RIO current
                    subsystem_current_draws["radio"] = rio_current * 0.5
                elif record.entry == key_ids["/Arm/ShoulderCurrentAmps"]:
                    subsystem_current_draws["shoulder"] = array_sum(record.getDoubleArray())
                elif record.entry == key_ids["/Arm/ElbowCurrentAmps"]:
                    subsystem_current_draws["elbow"] = array_sum(record.getDoubleArray())
                elif record.entry == key_ids["/Arm/WristCurrentAmps"]:
                    subsystem_current_draws["wrist"] = array_sum(record.getDoubleArray())
                elif record.entry == key_ids["/Gripper/CurrentAmps"]:
                    subsystem_current_draws["gripper"] = array_sum(record.getDoubleArray())
                elif record.entry == key_ids["/CubeIntake/ArmCurrentAmps"]:
                    subsystem_current_draws["intake_arm"] = array_sum(record.getDoubleArray())
                elif record.entry == key_ids["/CubeIntake/RollerCurrentAmps"]:
                    subsystem_current_draws["intake_roller"] = array_sum(record.getDoubleArray())
                elif record.entry == key_ids["/ArmSolver/ParameterHash"]:
                    parameter_hash = record.getInteger()
                    if parameter_hash != 0:
                        stats["arm_trajectories"] += 1
                elif record.entry == key_ids["/Arm/ShoulderRelativePositionRad"]:
                    position = record.getDouble()
                    delta_radians = abs(position - last_shoulder_position)
                    stats["shoulder_rotations"] += delta_radians / (2 * math.pi)
                    last_shoulder_position = position
                elif record.entry == key_ids["/Arm/ElbowRelativePositionRad"]:
                    position = record.getDouble()
                    delta_radians = abs(position - last_elbow_position)
                    stats["elbow_rotations"] += delta_radians / (2 * math.pi)
                    last_elbow_position = position
                elif record.entry == key_ids["/Arm/WristRelativePositionRad"]:
                    position = record.getDouble()
                    delta_radians = abs(position - last_wrist_position)
                    stats["wrist_rotations"] += delta_radians / (2 * math.pi)
                    last_wrist_position = position
                elif record.entry == key_ids["/CubeIntake/ArmRelativePositionRad"]:
                    position = record.getDouble()
                    delta_radians = abs(position - last_intake_position)
                    stats["intake_rotations"] += delta_radians / (2 * math.pi)
                    last_intake_position = position
                else:
                    # Drive currents
                    for i in range(4):
                        if record.entry == key_ids["/Drive/Module" + str(i) + "/DriveCurrentAmps"]:
                            subsystem_current_draws["drive"][i] = array_sum(record.getDoubleArray())
                        elif record.entry == key_ids["/Drive/Module" + str(i) + "/TurnCurrentAmps"]:
                            subsystem_current_draws["turn"][i] = array_sum(record.getDoubleArray())

                    # Drive positions
                    for i in range(4):
                        if record.entry == key_ids["/Drive/Module" + str(i) + "/DrivePositionRad"]:
                            drive_position_meters = record.getDouble() * WHEEL_RADIUS
                            if math.isnan(drive_position_meters):
                                continue
                            if last_drive_position[i] == None:
                                last_drive_position[i] = drive_position_meters
                            delta = abs(drive_position_meters - last_drive_position[i])
                            if abs(drive_position_meters) < 1.0e-3 or abs(drive_position_meters) > 1.0e6 or delta > MAX_WHEEL_DELTA:
                                continue
                            last_drive_position[i] = drive_position_meters
                            stats["distance_meters"][i] += delta
                            if enabled and (auto_mode or not joystick_drive):
                                stats["distance_meters_auto"][i] += delta

                    # Vision frames
                    for i in range(4):
                        if record.entry == key_ids["/AprilTagVision/Inst" + str(i) + "/FrameCount"]:
                            vision_frame_counts[i] = record.getInteger()

        stats["distance_meters_avg"] = (
            stats["distance_meters"][0] +
            stats["distance_meters"][1] +
            stats["distance_meters"][2] +
            stats["distance_meters"][3]
        ) / 4
        stats["distance_meters_auto_avg"] = (
            stats["distance_meters_auto"][0] +
            stats["distance_meters_auto"][1] +
            stats["distance_meters_auto"][2] +
            stats["distance_meters_auto"][3]
        ) / 4
        return stats


if __name__ == "__main__":
    # Calculate stats for each log
    all_logs = [x for x in os.listdir(FOLDER) if not x[0] == "."]
    pool = Pool()
    all_stats = list(tqdm.tqdm(pool.imap_unordered(get_stats, all_logs), total=len(all_logs)))
    all_stats = sorted(all_stats, key=lambda x: x["filename"])

    # Combine stats
    stats_total = {
        "loop_cycles": 0,
        "time_total": 0.0,
        "time_teleop": 0.0,
        "time_auto": 0.0,
        "subsystem_power": {
            "control_system": 0.0,
            "drive": 0.0,
            "turn": 0.0,
            "shoulder": 0.0,
            "elbow": 0.0,
            "wrist": 0.0,
            "gripper": 0.0,
            "intake_arm": 0.0,
            "intake_roller": 0.0
        },
        "distance_meters": [0.0, 0.0, 0.0, 0.0],
        "distance_meters_auto": [0.0, 0.0, 0.0, 0.0],
        "distance_meters_avg": 0.0,
        "distance_meters_auto_avg": 0.0,
        "vision_frames": 0,
        "arm_trajectories": 0,
        "shoulder_rotations": 0,
        "elbow_rotations": 0,
        "wrist_rotations": 0,
        "intake_rotations": 0
    }
    stats_matches = []
    stats_field_all = {
        "loop_cycles": 0,
        "time_total": 0.0,
        "time_teleop": 0.0,
        "time_auto": 0.0,
        "subsystem_power": {
            "control_system": 0.0,
            "drive": 0.0,
            "turn": 0.0,
            "shoulder": 0.0,
            "elbow": 0.0,
            "wrist": 0.0,
            "gripper": 0.0,
            "intake_arm": 0.0,
            "intake_roller": 0.0
        },
        "distance_meters": [0.0, 0.0, 0.0, 0.0],
        "distance_meters_auto": [0.0, 0.0, 0.0, 0.0],
        "distance_meters_avg": 0.0,
        "distance_meters_auto_avg": 0.0,
        "vision_frames": 0,
        "arm_trajectories": 0,
        "shoulder_rotations": 0,
        "elbow_rotations": 0,
        "wrist_rotations": 0,
        "intake_rotations": 0
    }
    stats_match_count = 0
    for stats in all_stats:
        if stats == None:
            continue
        stats_total["loop_cycles"] += stats["loop_cycles"]
        stats_total["time_total"] += stats["time_total"]
        stats_total["time_teleop"] += stats["time_teleop"]
        stats_total["time_auto"] += stats["time_auto"]
        for key, value in stats["subsystem_power"].items():
            stats_total["subsystem_power"][key] += value
        for i in range(4):
            stats_total["distance_meters"][i] += stats["distance_meters"][i]
            stats_total["distance_meters_auto"][i] += stats["distance_meters_auto"][i]
        stats_total["distance_meters_avg"] += stats["distance_meters_avg"]
        stats_total["distance_meters_auto_avg"] += stats["distance_meters_auto_avg"]
        stats_total["vision_frames"] += stats["vision_frames"]
        stats_total["arm_trajectories"] += stats["arm_trajectories"]
        stats_total["shoulder_rotations"] += stats["shoulder_rotations"]
        stats_total["elbow_rotations"] += stats["elbow_rotations"]
        stats_total["wrist_rotations"] += stats["wrist_rotations"]
        stats_total["intake_rotations"] += stats["intake_rotations"]
        if stats["is_match"]:
            stats_match_count += 1
            stats_matches.append(stats)
            stats_field_all["loop_cycles"] += stats["loop_cycles"]
            stats_field_all["time_total"] += stats["time_total"]
            stats_field_all["time_teleop"] += stats["time_teleop"]
            stats_field_all["time_auto"] += stats["time_auto"]
            for key, value in stats["subsystem_power"].items():
                stats_field_all["subsystem_power"][key] += value
            for i in range(4):
                stats_field_all["distance_meters"][i] += stats["distance_meters"][i]
                stats_field_all["distance_meters_auto"][i] += stats["distance_meters_auto"][i]
            stats_field_all["distance_meters_avg"] += stats["distance_meters_avg"]
            stats_field_all["distance_meters_auto_avg"] += stats["distance_meters_auto_avg"]
            stats_field_all["vision_frames"] += stats["vision_frames"]
            stats_field_all["arm_trajectories"] += stats["arm_trajectories"]
            stats_field_all["shoulder_rotations"] += stats["shoulder_rotations"]
            stats_field_all["elbow_rotations"] += stats["elbow_rotations"]
            stats_field_all["wrist_rotations"] += stats["wrist_rotations"]
            stats_field_all["intake_rotations"] += stats["intake_rotations"]

    # Calculate match averages
    stats_field_all["loop_cycles"] /= stats_match_count
    stats_field_all["time_total"] /= stats_match_count
    stats_field_all["time_teleop"] /= stats_match_count
    stats_field_all["time_auto"] /= stats_match_count
    for key in stats_field_all["subsystem_power"].keys():
        stats_field_all["subsystem_power"][key] /= stats_match_count
    for i in range(4):
        stats_field_all["distance_meters"][i] /= stats_match_count
        stats_field_all["distance_meters_auto"][i] /= stats_match_count
    stats_field_all["distance_meters_avg"] /= stats_match_count
    stats_field_all["distance_meters_auto_avg"] /= stats_match_count
    stats_field_all["vision_frames"] /= stats_match_count
    stats_field_all["arm_trajectories"] /= stats_match_count
    stats_field_all["shoulder_rotations"] /= stats_match_count
    stats_field_all["elbow_rotations"] /= stats_match_count
    stats_field_all["wrist_rotations"] /= stats_match_count
    stats_field_all["intake_rotations"] /= stats_match_count

    # Write CSV
    with open("results_matches.csv", "w") as csv:
        csv.write("Filename,Loop Cycles,Time (Total),Time (Teleop),Time (Auto),Distance 0 (Meters),Distance 1 (Meters),Distance 2 (Meters),Distance 3 (Meters),Auto Distance 0 (Meters),Auto Distance 1 (Meters),Auto Distance 2 (Meters),Auto Distance 3 (Meters),Avg Distance (Meters),Auto Avg Distance (Meters),Vision Frames,Arm Trajectory Count (Kairos),Shoulder Rotations,Elbow Rotations,Wrist Rotations,Intake Rotations,Power (Total),Power (Control System),Power (Drive),Power (Turn),Power (Shoulder),Power (Elbow),Power (Wrist),Power (Gripper),Power (Intake Arm),Power (Intake Roller)\n")
        for stats in stats_matches:
            csv.write(stats["filename"] + ",")
            csv.write(str(stats["loop_cycles"]) + ",")
            csv.write(str(stats["time_total"]) + ",")
            csv.write(str(stats["time_teleop"]) + ",")
            csv.write(str(stats["time_auto"]) + ",")
            for i in range(4):
                csv.write(str(stats["distance_meters"][i]) + ",")
            for i in range(4):
                csv.write(str(stats["distance_meters_auto"][i]) + ",")
            csv.write(str(stats["distance_meters_avg"]) + ",")
            csv.write(str(stats["distance_meters_auto_avg"]) + ",")
            csv.write(str(stats["vision_frames"]) + ",")
            csv.write(str(stats["arm_trajectories"]) + ",")
            csv.write(str(stats["shoulder_rotations"]) + ",")
            csv.write(str(stats["elbow_rotations"]) + ",")
            csv.write(str(stats["wrist_rotations"]) + ",")
            csv.write(str(stats["intake_rotations"]) + ",")
            csv.write(str(stats["total_power"]) + ",")
            csv.write(str(stats["subsystem_power"]["control_system"]) + ",")
            csv.write(str(stats["subsystem_power"]["drive"]) + ",")
            csv.write(str(stats["subsystem_power"]["turn"]) + ",")
            csv.write(str(stats["subsystem_power"]["shoulder"]) + ",")
            csv.write(str(stats["subsystem_power"]["elbow"]) + ",")
            csv.write(str(stats["subsystem_power"]["wrist"]) + ",")
            csv.write(str(stats["subsystem_power"]["gripper"]) + ",")
            csv.write(str(stats["subsystem_power"]["intake_arm"]) + ",")
            csv.write(str(stats["subsystem_power"]["intake_roller"]))
            csv.write("\n")

    print("\nTotal:")
    print(json.dumps(stats_total, indent=2))
    print("\nMatch Averages (" + str(stats_match_count) + " matches):")
    print(json.dumps(stats_field_all, indent=2))
