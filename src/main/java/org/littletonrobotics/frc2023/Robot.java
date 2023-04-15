// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2023;

import edu.wpi.first.hal.AllianceStationID;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;
import org.littletonrobotics.frc2023.Constants.Mode;
import org.littletonrobotics.frc2023.Constants.RobotType;
import org.littletonrobotics.frc2023.subsystems.leds.Leds;
import org.littletonrobotics.frc2023.util.Alert;
import org.littletonrobotics.frc2023.util.Alert.AlertType;
import org.littletonrobotics.frc2023.util.BatteryTracker;
import org.littletonrobotics.frc2023.util.VirtualSubsystem;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggedPowerDistribution;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
  private static final String batteryNameFile = "/home/lvuser/battery-name.txt";
  private static final double canErrorTimeThreshold = 0.5; // Seconds to disable alert
  private static final double lowBatteryVoltage = 10.0;
  private static final double lowBatteryDisabledTime = 1.5;

  private RobotContainer robotContainer;
  private Command autoCommand;
  private double autoStart;
  private boolean autoMessagePrinted;
  private boolean batteryNameWritten = false;
  private final Timer canErrorTimer = new Timer();
  private final Timer canErrorTimerInitial = new Timer();
  private final Timer disabledTimer = new Timer();

  private final Alert logNoFileAlert =
      new Alert("No log path set for current robot. Data will NOT be logged.", AlertType.WARNING);
  private final Alert logReceiverQueueAlert =
      new Alert("Logging queue exceeded capacity, data will NOT be logged.", AlertType.ERROR);
  private final Alert sameBatteryAlert =
      new Alert("The battery has not been changed since the last match.", AlertType.WARNING);
  private final Alert canErrorAlert =
      new Alert("CAN errors detected, robot may not be controllable.", AlertType.ERROR);
  private final Alert lowBatteryAlert =
      new Alert(
          "Battery voltage is very low, consider turning off the robot or replacing the battery.",
          AlertType.WARNING);

  public Robot() {
    super(Constants.loopPeriodSecs);
  }

  @Override
  public void robotInit() {
    Logger logger = Logger.getInstance();
    Leds.getInstance();

    // Record metadata
    logger.recordMetadata("Robot", Constants.getRobot().toString());
    System.out.println("[Init] Scanning battery");
    logger.recordMetadata("BatteryName", "BAT-" + BatteryTracker.scanBattery(1.5));
    System.out.println("[Init] Starting AdvantageKit");
    logger.recordMetadata("TuningMode", Boolean.toString(Constants.tuningMode));
    logger.recordMetadata("RuntimeType", getRuntimeType().toString());
    logger.recordMetadata("ProjectName", BuildConstants.MAVEN_NAME);
    logger.recordMetadata("BuildDate", BuildConstants.BUILD_DATE);
    logger.recordMetadata("GitSHA", BuildConstants.GIT_SHA);
    logger.recordMetadata("GitDate", BuildConstants.GIT_DATE);
    logger.recordMetadata("GitBranch", BuildConstants.GIT_BRANCH);
    switch (BuildConstants.DIRTY) {
      case 0:
        logger.recordMetadata("GitDirty", "All changes committed");
        break;
      case 1:
        logger.recordMetadata("GitDirty", "Uncomitted changes");
        break;
      default:
        logger.recordMetadata("GitDirty", "Unknown");
        break;
    }

    // Set up data receivers & replay source
    switch (Constants.getMode()) {
      case REAL:
        String folder = Constants.logFolders.get(Constants.getRobot());
        if (folder != null) {
          logger.addDataReceiver(new WPILOGWriter(folder));
        } else {
          logNoFileAlert.set(true);
        }
        logger.addDataReceiver(new NT4Publisher());
        if (Constants.getRobot() == RobotType.ROBOT_2023C) {
          LoggedPowerDistribution.getInstance(50, ModuleType.kRev);
        }
        break;

      case SIM:
        logger.addDataReceiver(new NT4Publisher());
        break;

      case REPLAY:
        String path = LogFileUtil.findReplayLog();
        logger.setReplaySource(new WPILOGReader(path));
        logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(path, "_sim")));
        break;
    }

    // Start AdvantageKit logger
    setUseTiming(Constants.getMode() != Mode.REPLAY);
    logger.start();

    // Check for battery alert
    if (Constants.getMode() == Mode.REAL
        && !BatteryTracker.getName().equals(BatteryTracker.defaultName)) {
      File file = new File(batteryNameFile);
      if (file.exists()) {
        // Read previous battery name
        String previousBatteryName = "";
        try {
          previousBatteryName =
              new String(Files.readAllBytes(Paths.get(batteryNameFile)), StandardCharsets.UTF_8);
        } catch (IOException e) {
          e.printStackTrace();
        }

        if (previousBatteryName.equals(BatteryTracker.getName())) {
          // Same battery, set alert
          sameBatteryAlert.set(true);
          Leds.getInstance().sameBattery = true;
        } else {
          // New battery, delete file
          file.delete();
        }
      }
    }

    // Log active commands
    Map<String, Integer> commandCounts = new HashMap<>();
    BiConsumer<Command, Boolean> logCommandFunction =
        (Command command, Boolean active) -> {
          String name = command.getName();
          int count = commandCounts.getOrDefault(name, 0) + (active ? 1 : -1);
          commandCounts.put(name, count);
          Logger.getInstance()
              .recordOutput(
                  "CommandsUnique/" + name + "_" + Integer.toHexString(command.hashCode()), active);
          Logger.getInstance().recordOutput("CommandsAll/" + name, count > 0);
        };
    CommandScheduler.getInstance()
        .onCommandInitialize(
            (Command command) -> {
              logCommandFunction.accept(command, true);
            });
    CommandScheduler.getInstance()
        .onCommandFinish(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            (Command command) -> {
              logCommandFunction.accept(command, false);
            });

    // Default to blue alliance in sim
    if (Constants.getMode() == Mode.SIM) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue1);
    }

    // Start timers
    canErrorTimer.reset();
    canErrorTimer.start();
    canErrorTimerInitial.reset();
    canErrorTimerInitial.start();
    disabledTimer.reset();
    disabledTimer.start();

    // Instantiate RobotContainer
    System.out.println("[Init] Instantiating RobotContainer");
    robotContainer = new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    Threads.setCurrentThreadPriority(true, 99);
    VirtualSubsystem.periodicAll();
    CommandScheduler.getInstance().run();

    // Check logging fault
    logReceiverQueueAlert.set(Logger.getInstance().getReceiverQueueFault());

    // Robot container periodic methods
    robotContainer.checkControllers();
    robotContainer.updateHPModeLeds();

    // Update CAN error alert
    var canStatus = RobotController.getCANStatus();
    if (canStatus.receiveErrorCount > 0 || canStatus.transmitErrorCount > 0) {
      canErrorTimer.reset();
    }
    canErrorAlert.set(
        !canErrorTimer.hasElapsed(canErrorTimeThreshold)
            && canErrorTimerInitial.hasElapsed(canErrorTimeThreshold));

    // Update low battery alert
    if (DriverStation.isEnabled()) {
      disabledTimer.reset();
    }
    if (RobotController.getBatteryVoltage() < lowBatteryVoltage
        && disabledTimer.hasElapsed(lowBatteryDisabledTime)) {
      Leds.getInstance().lowBatteryAlert = true;
      lowBatteryAlert.set(true);
    }

    // Log list of NT clients
    List<String> clientNames = new ArrayList<>();
    List<String> clientAddresses = new ArrayList<>();
    for (var client : NetworkTableInstance.getDefault().getConnections()) {
      clientNames.add(client.remote_id);
      clientAddresses.add(client.remote_ip);
    }
    Logger.getInstance()
        .recordOutput("NTClients/Names", clientNames.toArray(new String[clientNames.size()]));
    Logger.getInstance()
        .recordOutput(
            "NTClients/Addresses", clientAddresses.toArray(new String[clientAddresses.size()]));

    // Print auto duration
    if (autoCommand != null) {
      if (!autoCommand.isScheduled() && !autoMessagePrinted) {
        if (DriverStation.isAutonomousEnabled()) {
          System.out.println(
              String.format(
                  "*** Auto finished in %.2f secs ***", Timer.getFPGATimestamp() - autoStart));
        } else {
          System.out.println(
              String.format(
                  "*** Auto cancelled in %.2f secs ***", Timer.getFPGATimestamp() - autoStart));
        }
        autoMessagePrinted = true;
        Leds.getInstance().autoFinished = true;
        Leds.getInstance().autoFinishedTime = Timer.getFPGATimestamp();
      }
    }

    // Write battery name if connected to field
    if (Constants.getMode() == Mode.REAL
        && !batteryNameWritten
        && !BatteryTracker.getName().equals(BatteryTracker.defaultName)
        && DriverStation.isFMSAttached()) {
      batteryNameWritten = true;
      try {
        FileWriter fileWriter = new FileWriter(batteryNameFile);
        fileWriter.write(BatteryTracker.getName());
        fileWriter.close();
      } catch (IOException e) {
        e.printStackTrace();
      }
    }

    Threads.setCurrentThreadPriority(true, 10);
  }

  @Override
  public void autonomousInit() {
    autoStart = Timer.getFPGATimestamp();
    autoMessagePrinted = false;
    autoCommand = robotContainer.getAutonomousCommand();
    if (autoCommand != null) {
      autoCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (autoCommand != null) {
      autoCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
