// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023.util;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import edu.wpi.first.hal.CANData;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;

/**
 * Alternative system for reading and using the velocity measurements from the internal encoder on a
 * Spark Max. By default, the filtering of the velocity data produces a 112ms latency, making it
 * significantly more difficult to use feedback control for velocity. This class instead uses
 * position measurements (which have no filtering) to derive the velocity on the RIO and run the
 * control loop using a notifier thread. The period of the control loop and the number of averaging
 * taps are configurable. Note the following warnings:
 *
 * <p>Reading the internal encoder position from REVLib will be nonfunctional. All other functions
 * are unaffected.
 *
 * <p>The position conversion factor on the Spark Max will be automatically reset to 1 when
 * initialized. We recommend running unit conversions on the RIO instead ({@link #getPosition()}
 * returns rotations and {@link #getVelocity()} returns rotations/minute).
 *
 * <p>The units for the PID gains do not match the native units of the Spark Max. Expect to retune
 * these gains when switching to this class.
 */
public class SparkMaxDerivedVelocityController {
  private static final int deviceManufacturer = 5; // REV
  private static final int deviceType = 2; // Spark Max
  private static final int apiId = 98; // Periodic status 2

  private final CANSparkMax sparkMax;
  private final CAN canInterface;
  private final LinearFilter velocityFilter;
  private final PIDController velocityController;
  private final Notifier notifier;

  private boolean firstCycle = true;
  private boolean enabled = false;
  private double ffVolts = 0.0;
  private double timestamp = 0.0;
  private double position = 0.0;
  private double velocity = 0.0;

  /** Creates a new SparkMaxDerivedVelocityController using a default set of parameters. */
  public SparkMaxDerivedVelocityController(CANSparkMax sparkMax) {
    this(sparkMax, 0.02, 5);
  }

  /** Creates a new SparkMaxDerivedVelocityController. */
  public SparkMaxDerivedVelocityController(
      CANSparkMax sparkMax, double periodSeconds, int averagingTaps) {
    this.sparkMax = sparkMax;
    sparkMax.getEncoder().setPositionConversionFactor(1.0);
    int periodMs = (int) (periodSeconds * 1000);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, periodMs);

    canInterface = new CAN(sparkMax.getDeviceId(), deviceManufacturer, deviceType);
    velocityFilter = LinearFilter.movingAverage(averagingTaps);
    velocityController = new PIDController(0.0, 0.0, 0.0, periodSeconds);
    notifier = new Notifier(this::update);
    notifier.startPeriodic(periodSeconds);
  }

  /** Reads new data, updates the velocity measurement, and runs the controller. */
  private void update() {
    CANData canData = new CANData();
    boolean isFresh = canInterface.readPacketNew(apiId, canData);
    double newTimestamp = canData.timestamp / 1000.0;
    double newPosition =
        ByteBuffer.wrap(canData.data).order(ByteOrder.LITTLE_ENDIAN).asFloatBuffer().get(0);

    if (isFresh) {
      synchronized (this) {
        if (!firstCycle) {
          velocity =
              velocityFilter.calculate((newPosition - position) / (newTimestamp - timestamp) * 60);
        }
        firstCycle = false;
        timestamp = newTimestamp;
        position = newPosition;

        if (DriverStation.isDisabled()) {
          enabled = false;
          sparkMax.stopMotor();
        }
        if (enabled) {
          sparkMax.setVoltage(ffVolts + velocityController.calculate(velocity));
        }
      }
    }
  }

  /**
   * Sets the desired velocity and enables the controller. Note that the controller will continue to
   * run until {@link #disable()} is called or the robot is disabled, even if another command is
   * sent to the Spark Max.
   */
  public synchronized void setReference(double velocityRpm, double ffVolts) {
    velocityController.setSetpoint(velocityRpm);
    this.ffVolts = ffVolts;

    if (!enabled) {
      velocityController.reset();
    }
    enabled = true;
  }

  /**
   * Disables the controller. No further commands will be sent to the Spark Max, but the
   * measurements will continue to update.
   */
  public synchronized void disable() {
    if (enabled) {
      sparkMax.stopMotor();
    }
    enabled = false;
  }

  /** Sets the PID gains. */
  public synchronized void setPID(double kp, double ki, double kd) {
    velocityController.setPID(kp, ki, kd);
  }

  /** Returns the current position in rotations. */
  public synchronized double getPosition() {
    return position;
  }

  /** Returns the current velocity in rotations/minute. */
  public synchronized double getVelocity() {
    return velocity;
  }
}
