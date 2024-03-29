// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

/** Helper class for creating a {@link Mechanism2d} and 3D component representation of an arm. */
public class ArmVisualizer {
  private final ArmConfig config;
  private final String logKey;

  private final Mechanism2d mechanism;
  private final MechanismRoot2d mechanismRoot;
  private final MechanismLigament2d fixedLigament;
  private final MechanismLigament2d shoulderLigament;
  private final MechanismLigament2d elbowLigament;
  private final MechanismLigament2d wristLigament;

  public ArmVisualizer(ArmConfig config, String logKey, Color8Bit colorOverride) {
    this.config = config;
    this.logKey = logKey;
    mechanism = new Mechanism2d(4, 3, new Color8Bit(Color.kGray));
    mechanismRoot = mechanism.getRoot("Arm", 2 + config.origin().getX(), 0);
    fixedLigament =
        mechanismRoot.append(
            new MechanismLigament2d(
                "Fixed", config.origin().getY(), 90, 6, new Color8Bit(Color.kBlack)));
    shoulderLigament =
        fixedLigament.append(
            new MechanismLigament2d(
                "Shoulder",
                config.shoulder().length(),
                0,
                4,
                colorOverride != null ? colorOverride : new Color8Bit(Color.kDarkBlue)));
    elbowLigament =
        shoulderLigament.append(
            new MechanismLigament2d(
                "Elbow",
                config.elbow().length(),
                0,
                4,
                colorOverride != null ? colorOverride : new Color8Bit(Color.kBlue)));
    wristLigament =
        elbowLigament.append(
            new MechanismLigament2d(
                "Wrist",
                config.wrist().length(),
                0,
                4,
                colorOverride != null ? colorOverride : new Color8Bit(Color.kSkyBlue)));
  }

  public void update(double shoulderAngle, double elbowAngle, double wristAngle) {
    shoulderLigament.setAngle(Units.radiansToDegrees(shoulderAngle) - 90.0);
    elbowLigament.setAngle(Units.radiansToDegrees(elbowAngle));
    wristLigament.setAngle(Units.radiansToDegrees(wristAngle));
    Logger.getInstance().recordOutput("Mechanism2d/" + logKey, mechanism);

    var shoulderPose =
        new Pose3d(
            config.origin().getX(),
            0.0,
            config.origin().getY(),
            new Rotation3d(0.0, -shoulderAngle, 0.0));
    var elbowPose =
        shoulderPose.transformBy(
            new Transform3d(
                new Translation3d(config.shoulder().length(), 0.0, 0.0),
                new Rotation3d(0.0, -elbowAngle, 0.0)));
    var wristPose =
        elbowPose.transformBy(
            new Transform3d(
                new Translation3d(config.elbow().length(), 0.0, 0.0),
                new Rotation3d(0.0, -wristAngle, 0.0)));
    Logger.getInstance().recordOutput("Mechanism3d/" + logKey, shoulderPose, elbowPose, wristPose);
  }

  /** Logs the rectangle constraints in the provided config as a 2D mechanism. */
  public static void logRectangleConstraints(String logKey, ArmConfig config, Color8Bit color) {
    List<ArmConfig.Constraint> rectangleConstraints = new ArrayList<>();
    for (var constraint : config.constraints().values()) {
      if (constraint.type().equals("rectangle")) {
        rectangleConstraints.add(constraint);
      }
    }

    double[][] rectangles = new double[rectangleConstraints.size()][];
    for (int i = 0; i < rectangleConstraints.size(); i++) {
      rectangles[i] = rectangleConstraints.get(i).args();
    }

    logRectangles(logKey, rectangles, color);
  }

  /** Logs a set of rectangles as a 2D mechanism. */
  public static void logRectangles(String logKey, double[][] rects, Color8Bit color) {
    var mechanism = new Mechanism2d(4, 3, new Color8Bit(Color.kGray));

    for (int i = 0; i < rects.length; i++) {
      var rect = rects[i];
      var root = mechanism.getRoot("Rect" + Integer.toString(i), 2 + rect[0], rect[1]);
      var bottomLigament =
          root.append(new MechanismLigament2d("Bottom", rect[2] - rect[0], 0, 1, color));
      var rightLigament =
          bottomLigament.append(new MechanismLigament2d("Right", rect[3] - rect[1], 90, 1, color));
      var topLigament =
          rightLigament.append(new MechanismLigament2d("Top", rect[2] - rect[0], 90, 1, color));
      topLigament.append(new MechanismLigament2d("Left", rect[3] - rect[1], 90, 1, color));
    }

    Logger.getInstance().recordOutput("Mechanism2d/" + logKey, mechanism);
  }
}
