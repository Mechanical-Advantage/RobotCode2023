# Copyright (c) 2023 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import json
import time

import ntcore

from Solver import Solver

SERVER_IP = "127.0.0.1"
DEVICE_ID = 0

solver = None
result_sub = None


def config_update(event):
    global solver, result_sub
    print("Creating new solver...")
    config = json.loads(event.data.value.getString())
    solver = Solver(config)
    print("Solver ready")


def request_update(event):
    global solver, result_sub
    if solver == None:
        print("Skipping request, solver not available")
    print("Solving trajectory...")
    parameters = json.loads(event.data.value.getString())
    start_time = time.time()
    result = solver.solve(parameters)
    end_time = time.time()
    print()
    if result != None:
        print("Trajectory generated")
        print("\tGenTime = " + str(end_time - start_time))
        print("\tPathTime = " + str(result[0]))
        print("\tDT = " + str(result[0] / (len(result[1]) - 1)))
        points = [parameters["hash"], result[0]]
        for i in range(len(result[1])):
            points.append(result[1][i])
            points.append(result[2][i])
        result_sub.set(points)
    else:
        print("Trajectory generation failed")


if __name__ == "__main__":
    # Set up NT instance
    nt_inst = ntcore.NetworkTableInstance.getDefault()
    nt_inst.setServer(SERVER_IP)
    nt_inst.startClient4("kairos_" + str(DEVICE_ID))

    # Create subscribers and publisher
    config_sub = nt_inst.getStringTopic("/kairos/config").subscribe(
        "", ntcore.PubSubOptions(periodic=0)
    )
    request_sub = nt_inst.getStringTopic("/kairos/request").subscribe(
        "", ntcore.PubSubOptions(periodic=0)
    )
    result_sub = nt_inst.getDoubleArrayTopic(
        "/kairos/result/" + str(DEVICE_ID)
    ).publish(ntcore.PubSubOptions(periodic=0))

    # Create event listeners
    nt_inst.addListener(config_sub, ntcore.EventFlags.kValueRemote, config_update)
    nt_inst.addListener(request_sub, ntcore.EventFlags.kValueRemote, request_update)

    # Run forever
    while True:
        time.sleep(1)
