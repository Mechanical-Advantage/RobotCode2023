// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

/** Preset configurations for Spark Max periodic frame rates. */
public class SparkMaxPeriodicFrameConfig {
  public static void configNotLeader(CANSparkMax sparkMax) {
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
  }

  public static void configLeaderFollower(CANSparkMax sparkMax) {
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 10);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus4, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus5, 65535);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus6, 65535);
  }
}
