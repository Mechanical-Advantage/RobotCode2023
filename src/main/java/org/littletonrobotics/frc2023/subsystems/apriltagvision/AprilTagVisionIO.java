// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.apriltagvision;

import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

public interface AprilTagVisionIO {
  public static class AprilTagVisionIOInputs implements LoggableInputs {
    public double[] timestamps = new double[] {};
    public double[][] frames = new double[][] {};
    public long fps = 0;

    @Override
    public void toLog(LogTable table) {
      table.put("Timestamps", timestamps);
      table.put("FrameCount", frames.length);
      for (int i = 0; i < frames.length; i++) {
        table.put("Frame/" + Integer.toString(i), frames[i]);
      }
      table.put("Fps", fps);
    }

    @Override
    public void fromLog(LogTable table) {
      timestamps = table.getDoubleArray("Timestamps", timestamps);
      int frameCount = (int) table.getInteger("FrameCount", 0);
      frames = new double[frameCount][];
      for (int i = 0; i < frameCount; i++) {
        frames[i] = table.getDoubleArray("Frame/" + Integer.toString(i), new double[] {});
      }
      fps = table.getInteger("Fps", fps);
    }
  }

  public default void updateInputs(AprilTagVisionIOInputs inputs) {}
}
