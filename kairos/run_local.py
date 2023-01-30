# Copyright (c) 2023 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import json

from Plotter import plot
from Solver import Solver

if __name__ == "__main__":
    config = json.loads(open("src/main/deploy/arm_config.json", "r").read())
    solver = Solver(config)

    request = {
        "hash": -1879723957,
        "initialJointPositions": [1.4864978511506446, 5.658764589965396],
        "finalJointPositions": [2.098698247638489, 1.9481554011761955],
        "initialJointVelocities": [0.0, 0.0],
        "finalJointVelocities": [0.0, 0.0],
        "constraintKeys": ["testRectangle"],
    }
    result = solver.solve(request)

    if result != None:
        print("DT =", result[0] / (len(result[1]) - 1))
        plot(result, config)
