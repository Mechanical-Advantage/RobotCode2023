# Copyright (c) 2023 FRC 6328
# http://github.com/Mechanical-Advantage
#
# Use of this source code is governed by an MIT-style
# license that can be found in the LICENSE file at
# the root directory of this project.

import json
from math import pi

from Plotter import plot
from Solver import Solver

if __name__ == "__main__":
    config = json.loads(open("src/main/deploy/arm_config.json", "r").read())
    solver = Solver(config)

    request = {
        "initial": [pi - (pi / 2.5), pi / 2],
        "final": [pi / 2.5, 3 * pi / 2],
        "constraintOverrides": [],
    }
    result = solver.solve(request)

    if result != None:
        print("DT =", result[0] / (len(result[1]) - 1))
        plot(result, config)
