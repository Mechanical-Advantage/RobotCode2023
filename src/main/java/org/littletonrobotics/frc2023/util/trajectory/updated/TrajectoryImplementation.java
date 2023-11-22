// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.util.trajectory.updated;

import edu.wpi.first.math.geometry.Pose2d;

public interface TrajectoryImplementation {
  double getDuration();

  Pose2d[] getTrajectoryPoses();

  CustomSwerveDriveController.DriveDynamicState sample(double time);
}
