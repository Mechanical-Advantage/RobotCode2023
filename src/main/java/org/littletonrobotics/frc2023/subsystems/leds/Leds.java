// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.subsystems.leds;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import org.littletonrobotics.frc2023.util.VirtualSubsystem;

public class Leds extends VirtualSubsystem {

  private static Leds instance;

  public static Leds getInstance() {
    if (instance == null) {
      instance = new Leds();
    }
    return instance;
  }

  // Robot state tracking
  public int loopCycleCount = 0;
  public boolean hpCone = false;
  public boolean hpConeTipped = false;
  public boolean hpDoubleSubstation = false;
  public boolean hpThrowGamePiece = false;
  public boolean autoScore = false;
  public boolean autoSubstation = false;
  public boolean distraction = false;
  public boolean fallen = false;
  public boolean sameBattery = false;
  public boolean armCoast = false;
  public boolean armEstopped = false;
  private Alliance alliance = Alliance.Invalid;

  // LED IO
  private final AddressableLED leds;
  private final AddressableLEDBuffer buffer;

  // Constants
  private static final int minLoopCycleCount = 10;
  private static final int length = 43;
  private static final int staticLength = 14;
  private static final int staticSectionLength = 5;
  private static final double strobeFastDuration = 0.1;
  private static final double strobeSlowDuration = 0.2;
  private static final double breathDuration = 1.0;
  private static final double rainbowCycleLength = 25.0;
  private static final double rainbowDuration = 0.25;
  private static final double waveExponent = 0.4;
  private static final double waveFastCycleLength = 25.0;
  private static final double waveFastDuration = 0.25;
  private static final double waveSlowCycleLength = 25.0;
  private static final double waveSlowDuration = 3.0;
  private static final double waveAllianceCycleLength = 15.0;
  private static final double waveAllianceDuration = 2.0;

  private Leds() {
    leds = new AddressableLED(0);
    buffer = new AddressableLEDBuffer(length);
    leds.setLength(length);
    leds.setData(buffer);
    leds.start();
  }

  public void periodic() {
    // Update alliance color
    if (DriverStation.isFMSAttached()) {
      alliance = DriverStation.getAlliance();
    }

    // Exit during initial cycles
    loopCycleCount += 1;
    if (loopCycleCount < minLoopCycleCount) {
      return;
    }

    // Select LED mode
    solid(Section.FULL, Color.kBlack); // Default to off
    if (DriverStation.isEStopped()) {
      solid(Section.FULL, Color.kRed);
    } else if (DriverStation.isDisabled()) {
      switch (alliance) {
        case Red:
          wave(
              Section.FULL,
              Color.kRed,
              Color.kBlack,
              waveAllianceCycleLength,
              waveAllianceDuration);
          break;
        case Blue:
          wave(
              Section.FULL,
              Color.kBlue,
              Color.kBlack,
              waveAllianceCycleLength,
              waveAllianceDuration);
          break;
        default:
          wave(Section.FULL, Color.kGold, Color.kDarkBlue, waveSlowCycleLength, waveSlowDuration);
          break;
      }
    } else if (fallen || distraction) {
      strobe(Section.FULL, Color.kWhite, strobeFastDuration);
    } else if (DriverStation.isAutonomous()) {
      wave(Section.FULL, Color.kGold, Color.kDarkBlue, waveFastCycleLength, waveFastDuration);
    } else {
      // Set HP indicator
      Color hpColor = Color.kBlack;
      if (hpCone) {
        if (hpConeTipped) {
          hpColor = Color.kRed;
        } else {
          hpColor = Color.kGold;
        }
      } else {
        hpColor = Color.kPurple;
      }
      if (hpDoubleSubstation) {
        solid(Section.STATIC_LOW, hpColor);
        solid(Section.STATIC_HIGH, hpColor);
      } else if (hpThrowGamePiece) {
        strobe(Section.STATIC, hpColor, strobeSlowDuration);
      } else {
        solid(Section.STATIC, hpColor);
      }

      // Set special modes
      if (autoScore || autoSubstation) {
        rainbow(Section.SHOULDER, rainbowCycleLength, rainbowDuration);
      }
    }

    // Alerts at static base
    if (sameBattery) {
      breath(Section.STATIC_LOW, Color.kRed, Color.kBlack, breathDuration);
    } else if (armCoast) {
      solid(Section.STATIC_LOW, Color.kWhite);
    }

    // Arm estop alert
    if (armEstopped) {
      breath(Section.SHOULDER, Color.kRed, Color.kBlack, breathDuration);
    }

    // Update LEDs
    leds.setData(buffer);
  }

  private void solid(Section section, Color color) {
    for (int i = section.start(); i < section.end(); i++) {
      buffer.setLED(i, color);
    }
  }

  private void strobe(Section section, Color color, double duration) {
    boolean on = ((Timer.getFPGATimestamp() % duration) / duration) > 0.5;
    solid(section, on ? color : Color.kBlack);
  }

  private void breath(Section section, Color c1, Color c2, double duration) {
    double x = ((Timer.getFPGATimestamp() % breathDuration) / breathDuration) * 2.0 * Math.PI;
    double ratio = (Math.sin(x) + 1.0) / 2.0;
    double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
    double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
    double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
    solid(section, new Color(red, green, blue));
  }

  private void rainbow(Section section, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() / duration) % 1.0)) * 180.0;
    double xDiffPerLed = 180.0 / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      x %= 180.0;
      if (i >= section.start()) {
        buffer.setHSV(i, (int) x, 255, 255);
      }
    }
  }

  private void wave(Section section, Color c1, Color c2, double cycleLength, double duration) {
    double x = (1 - ((Timer.getFPGATimestamp() % duration) / duration)) * 2.0 * Math.PI;
    double xDiffPerLed = (2.0 * Math.PI) / cycleLength;
    for (int i = 0; i < section.end(); i++) {
      x += xDiffPerLed;
      if (i >= section.start()) {
        double ratio = (Math.pow(Math.sin(x), waveExponent) + 1.0) / 2.0;
        if (Double.isNaN(ratio)) {
          ratio = (-Math.pow(Math.sin(x + Math.PI), waveExponent) + 1.0) / 2.0;
        }
        if (Double.isNaN(ratio)) {
          ratio = 0.5;
        }
        double red = (c1.red * (1 - ratio)) + (c2.red * ratio);
        double green = (c1.green * (1 - ratio)) + (c2.green * ratio);
        double blue = (c1.blue * (1 - ratio)) + (c2.blue * ratio);
        buffer.setLED(i, new Color(red, green, blue));
      }
    }
  }

  private static enum Section {
    STATIC,
    SHOULDER,
    FULL,
    STATIC_LOW,
    STATIC_MID,
    STATIC_HIGH;

    private int start() {
      switch (this) {
        case STATIC:
          return 0;
        case SHOULDER:
          return staticLength;
        case FULL:
          return 0;
        case STATIC_LOW:
          return 0;
        case STATIC_MID:
          return staticSectionLength;
        case STATIC_HIGH:
          return staticLength - staticSectionLength;
        default:
          return 0;
      }
    }

    private int end() {
      switch (this) {
        case STATIC:
          return staticLength;
        case SHOULDER:
          return length;
        case FULL:
          return length;
        case STATIC_LOW:
          return staticSectionLength;
        case STATIC_MID:
          return staticLength - staticSectionLength;
        case STATIC_HIGH:
          return staticLength;
        default:
          return length;
      }
    }
  }
}
