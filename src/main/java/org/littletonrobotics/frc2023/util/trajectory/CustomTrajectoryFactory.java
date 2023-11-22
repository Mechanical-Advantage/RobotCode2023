// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.util.trajectory;

import static org.littletonrobotics.frc2023.util.trajectory.CustomSwerveDriveController.DriveDynamicState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import java.util.List;
import org.littletonrobotics.frc2023.util.AllianceFlipUtil;

public class CustomTrajectoryFactory {
  public static TrajectoryImplementation createFromWaypoints(
      TrajectoryConfig config, List<Waypoint> waypoints) {
    CustomTrajectoryGenerator trajectoryGenerator = new CustomTrajectoryGenerator();
    trajectoryGenerator.generate(config, waypoints);

    return new TrajectoryImplementation() {
      @Override
      public double getDuration() {
        return trajectoryGenerator.getDriveTrajectory().getTotalTimeSeconds();
      }

      @Override
      public Pose2d[] getTrajectoryPoses() {
        return trajectoryGenerator.getDriveTrajectory().getStates().stream()
            .map(state -> AllianceFlipUtil.apply(state.poseMeters))
            .toArray(Pose2d[]::new);
      }

      @Override
      public DriveDynamicState sample(double time) {
        Trajectory.State driveState = trajectoryGenerator.getDriveTrajectory().sample(time);
        RotationSequence.State rotationState =
            trajectoryGenerator.getHolonomicRotationSequence().sample(time);

        // TODO: Check if this is actually what I need to do
        double velocityX =
            driveState.poseMeters.getRotation().getCos() * driveState.velocityMetersPerSecond;
        double velocityY =
            driveState.poseMeters.getRotation().getSin() * driveState.velocityMetersPerSecond;

        return new DriveDynamicState(
            new Pose2d(driveState.poseMeters.getTranslation(), rotationState.position),
            velocityX,
            velocityY,
            rotationState.velocityRadiansPerSec);
      }
    };
  }
}
