// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.util;

import edu.wpi.first.wpilibj.RobotBase;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import org.littletonrobotics.frc2023.BuildConstants;

/** Determines whether to burn SparkMax configs to flash. */
public class SparkMaxBurnManager {
  private static final String buildDateFile = "/home/lvuser/build-date.txt";
  private static boolean shouldBurn = false;

  private SparkMaxBurnManager() {}

  public static boolean shouldBurn() {
    return shouldBurn;
  }

  public static void update() {
    if (RobotBase.isSimulation()) {
      shouldBurn = false;
      return;
    }

    File file = new File(buildDateFile);
    if (!file.exists()) {

      // No build date file, burn flash
      shouldBurn = true;
    } else {

      // Read previous build date
      String previousBuildDate = "";
      try {
        previousBuildDate =
            new String(Files.readAllBytes(Paths.get(buildDateFile)), StandardCharsets.UTF_8);
      } catch (IOException e) {
        e.printStackTrace();
      }

      shouldBurn = !previousBuildDate.equals(BuildConstants.BUILD_DATE);
    }

    try {
      FileWriter fileWriter = new FileWriter(buildDateFile);
      fileWriter.write(BuildConstants.BUILD_DATE);
      fileWriter.close();
    } catch (IOException e) {
      e.printStackTrace();
    }

    if (shouldBurn) {
      System.out.println("[SparkMaxBurnManager] Build date changed, burning SparkMax flash");
    } else {
      System.out.println(
          "[SparkMaxBurnManager] Build date unchanged, will not burn SparkMax flash");
    }
  }
}
