# Copyright (c) 2023 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import json
import multiprocessing
import os
import sys
import tempfile
import time

from Solver import Solver

solver = None


def calculate_func(trajectory):
    global solver

    # Create solver on first run
    if solver == None:
        config = json.loads(
            open(os.path.join("src", "main", "deploy", "arm_config.json"), "r").read()
        )
        solver = Solver(config, silence=True)

    # Generate trajectory
    return solver.solve(
        {
            "initial": trajectory["initialJointPositions"],
            "final": trajectory["finalJointPositions"],
            "constraintOverrides": [],
        }
    )


if __name__ == "__main__":
    start_time = time.time()
    cache_data = None
    with open(
        os.path.join(tempfile.gettempdir(), "arm_trajectory_cache_request.json"), "r"
    ) as cache_file:
        cache_data = json.loads(cache_file.read())

    # Generate all trajectories
    fail_count = 0
    results = multiprocessing.Pool().map(
        calculate_func,
        cache_data["trajectories"],
    )
    for i in range(len(results)):
        result = results[i]
        trajectory = cache_data["trajectories"][i]
        if result == None:
            print("Failed to generate trajectory:", trajectory)
            fail_count += 1
        else:
            trajectory["totalTime"] = result[0]
            trajectory["points"] = []
            for i in range(len(result[1])):
                trajectory["points"].append(result[1][i])
                trajectory["points"].append(result[2][i])

    # Save to JSON file
    if fail_count == 0:
        with open(
            os.path.join("src", "main", "deploy", "arm_trajectory_cache.json"), "w"
        ) as cache_file:
            cache_file.write(json.dumps(cache_data, separators=(",", ":")))

    # Print result
    end_time = time.time()
    cpu_count = multiprocessing.cpu_count()
    extra_info = (
        " ("
        + str(round((end_time - start_time) * 1000) / 1000)
        + " secs, "
        + str(cpu_count)
        + " "
        + ("core" if cpu_count == 1 else "cores")
        + ")"
    )
    if fail_count == 0:
        print("Successfully generated all trajectories" + extra_info)
        sys.exit(0)
    else:
        print("Failed to generate all trajectories" + extra_info)
        sys.exit(1)
