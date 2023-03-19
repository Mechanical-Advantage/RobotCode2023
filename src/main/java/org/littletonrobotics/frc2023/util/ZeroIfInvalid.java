// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.util;

public class ZeroIfInvalid {
  /** Returns zero if the value is NaN or infinite, otherwise returns the original value. */
  public static double zeroIfInvalid(double value) {
    return Double.isNaN(value) || Double.isInfinite(value) ? 0.0 : value;
  }
}
