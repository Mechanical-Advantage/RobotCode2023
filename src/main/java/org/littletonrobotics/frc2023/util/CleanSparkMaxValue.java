// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.util;

public class CleanSparkMaxValue {
  private static final double maxDiff = 60.0;

  public static double cleanSparkMaxValue(double lastValue, double value) {
    if (Double.isNaN(value)
        || Double.isInfinite(value)
        || (value == 0.0 && Math.abs(lastValue) > maxDiff)) {
      return lastValue;
    } else {
      return value;
    }
  }
}
