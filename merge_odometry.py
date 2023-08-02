import os
from datalog import *
import mmap
import ntcore
import math
import time


FOLDER = ""
DT = 0.02
MATCH_LENGTH = 153
FIELD_LENGTH = 651.25 * 0.0254

if __name__ == "__main__":
    ntcore.NetworkTableInstance.getDefault().setServer("127.0.0.1")
    ntcore.NetworkTableInstance.getDefault().startClient4("Python")
    publisher = ntcore.NetworkTableInstance.getDefault().getDoubleArrayTopic(
        "/Poses").publish(ntcore.PubSubOptions(periodic=0, sendAll=True))

    all_data = []
    while len(all_data) * DT < MATCH_LENGTH:
        all_data.append([])

    count = 0
    all_logs = [x for x in os.listdir(FOLDER) if not x[0] == "."]
    for filename in all_logs:
        count += 1
        print(filename + " (" + str(count) + "/" + str(len(all_logs)) + ")")
        with open(os.path.join(FOLDER, filename), "r") as f:
            mm = mmap.mmap(f.fileno(), 0, access=mmap.ACCESS_READ)
            reader = DataLogReader(mm)
            if not reader:
                print("Failed to parse", filename)
            else:
                enabled_id = None
                enabled_time = None
                alliance_id = None
                is_blue = False
                odometry_id = None
                i = 0
                for record in reader:
                    timestamp = record.timestamp / 1000000
                    if record.isStart():
                        data = record.getStartData()
                        if data.name == "/DriverStation/Enabled":
                            enabled_id = data.entry
                        elif data.name == "/DriverStation/AllianceStation":
                            alliance_id = data.entry
                        elif data.name == "/RealOutputs/Odometry/Robot":
                            odometry_id = data.entry

                    elif not record.isControl():
                        if alliance_id != None and record.entry == alliance_id:
                            is_blue = record.getInteger() > 2
                        if enabled_time == None:
                            if enabled_id != None and record.entry == enabled_id and record.getBoolean():
                                enabled_time = timestamp
                        else:
                            if odometry_id != None and record.entry == odometry_id:
                                value = record.getDoubleArray()
                                if is_blue:
                                    value = [FIELD_LENGTH - value[0], value[1], math.pi - value[2]]
                                while i < len(all_data) and i * DT < timestamp - enabled_time:
                                    all_data[i] += value
                                    i += 1

    print("Publishing on enter")
    input()
    start_time = time.time()
    for i in range(len(all_data)):
        timestamp = i * DT + start_time
        wait_time = timestamp - time.time()
        if wait_time > 0:
            time.sleep(wait_time)
        publisher.set(all_data[i], int(timestamp * 1000000))
