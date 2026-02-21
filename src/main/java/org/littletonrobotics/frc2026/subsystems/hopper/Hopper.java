// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.hopper;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.Constants.Mode;
import org.littletonrobotics.frc2026.RobotContainer.SimFuelCount;
import org.littletonrobotics.frc2026.subsystems.leds.Leds;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystem;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2026.subsystems.sensors.FuelSensorIO;
import org.littletonrobotics.frc2026.subsystems.sensors.FuelSensorIOInputsAutoLogged;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.LoggedTracer;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Hopper extends FullSubsystem {
  private static final LoggedTunableNumber rollerLaunchVolts =
      new LoggedTunableNumber("Hopper/Roller/LaunchVolts", 4.0);
  private static final LoggedTunableNumber rollerOuttakeVolts =
      new LoggedTunableNumber("Hopper/Roller/OuttakeVolts", -12.0);

  // Hopper depth meters
  private static final LoggedTunableNumber fuelDebounceTime =
      new LoggedTunableNumber("Hopper/LaserCan/DebounceTime", 1.0);
  private static final double[] hopperDepths = {0.5842, 0.4842, 0.43815, 0.2921, 0.14605, 0.0};
  private final Debouncer[] debouncers = {
    new Debouncer(fuelDebounceTime.get(), DebounceType.kRising),
    new Debouncer(fuelDebounceTime.get(), DebounceType.kRising),
    new Debouncer(fuelDebounceTime.get(), DebounceType.kRising),
    new Debouncer(fuelDebounceTime.get(), DebounceType.kRising),
    new Debouncer(fuelDebounceTime.get(), DebounceType.kRising)
  };

  private double maxRawDepth;

  // Length of filter time / loop cycle time
  private static final MedianFilter fuelFilter =
      new MedianFilter((int) (5.0 / Constants.loopPeriodSecs));

  private final RollerSystem roller;
  private final FuelSensorIO sensorLeft;
  private final FuelSensorIO sensorRight;
  private final FuelSensorIOInputsAutoLogged sensorLeftInputs = new FuelSensorIOInputsAutoLogged();
  private final FuelSensorIOInputsAutoLogged sensorRightInputs = new FuelSensorIOInputsAutoLogged();

  @Setter private BooleanSupplier coastOverride = () -> false;
  private final Optional<SimFuelCount> simFuelCount;

  @Getter @Setter @AutoLogOutput private Goal goal = Goal.STOP;
  @Getter @AutoLogOutput private HopperLevel hopperLevel = HopperLevel.EMPTY;

  public Hopper(
      RollerSystemIO rollerIO,
      FuelSensorIO fuelSensorIOLeft,
      FuelSensorIO fuelSensorIORight,
      Optional<SimFuelCount> simFuelCount) {
    this.roller = new RollerSystem("Hopper roller", "Hopper/Roller", rollerIO);
    this.sensorLeft = fuelSensorIOLeft;
    this.sensorRight = fuelSensorIORight;
    this.simFuelCount = simFuelCount;
    roller.setCoastOverride(coastOverride);
  }

  public void periodic() {
    sensorLeft.updateInputs(sensorLeftInputs);
    sensorRight.updateInputs(sensorRightInputs);
    Logger.processInputs("Hopper/Sensors/LeftSensor", sensorLeftInputs);
    Logger.processInputs("Hopper/Sensors/RightSensor", sensorRightInputs);
    roller.periodic();

    // Update hopper level
    if (sensorLeftInputs.valid && !sensorRightInputs.valid) {
      maxRawDepth = sensorLeftInputs.distanceMeters;
    } else if (!sensorLeftInputs.valid && sensorRightInputs.valid) {
      maxRawDepth = sensorRightInputs.distanceMeters;
    } else if (sensorLeftInputs.valid && sensorRightInputs.valid) {
      maxRawDepth = Math.max(sensorLeftInputs.distanceMeters, sensorRightInputs.distanceMeters);
    }

    double filteredMaxDepth = fuelFilter.calculate(maxRawDepth);
    Logger.recordOutput("Hopper/LaserCan/MaxRawDepth", maxRawDepth);
    Logger.recordOutput("Hopper/LaserCan/FilteredMaxDepth", filteredMaxDepth);

    // Debouncer filtered depth
    if (Constants.getMode() == Mode.SIM && simFuelCount.isPresent()) {
      double fuelPercent = (double) simFuelCount.get().getFuelStored() / SimFuelCount.getCapacity();
      if (simFuelCount.get().getFuelStored() == 0) {
        hopperLevel = HopperLevel.EMPTY;
      } else if (fuelPercent <= 0.25) {
        hopperLevel = HopperLevel.FULL_0_25;
      } else if (fuelPercent <= 0.5) {
        hopperLevel = HopperLevel.FULL_25_50;
      } else if (fuelPercent <= 0.75) {
        hopperLevel = HopperLevel.FULL_50_75;
      } else {
        hopperLevel = HopperLevel.FULL_75_100;
      }
    } else {
      for (int i = 0; i < hopperDepths.length - 1; i++) {
        if (filteredMaxDepth < hopperDepths[i] && filteredMaxDepth >= hopperDepths[i + 1]) {
          if (debouncers[i].calculate(true)) {
            hopperLevel = HopperLevel.values()[i];
          }
          for (int d = 0; d < debouncers.length; d++) {
            if (d != i) {
              debouncers[d].calculate(false);
            }
          }
        }
      }
    }

    // Send hopper level
    Leds.getGlobal().hopperLevel = hopperLevel;
    SmartDashboard.putString("Hopper Level", hopperLevel.toString());
    if (simFuelCount.isPresent()) {
      Logger.recordOutput("Hopper/SimFuelStored", simFuelCount.get().getFuelStored());
    }

    // Update roller output
    double rollerVolts = 0.0;
    switch (goal) {
      case LAUNCH -> {
        rollerVolts = rollerLaunchVolts.get();
      }
      case OUTTAKE -> {
        rollerVolts = rollerOuttakeVolts.get();
      }
      case STOP -> {
        rollerVolts = 0.0;
      }
    }

    roller.runOpenLoop(rollerVolts);
    LoggedTracer.record("Hopper/Periodic");
  }

  @Override
  public void periodicAfterScheduler() {
    roller.periodicAfterScheduler();
    LoggedTracer.record("Hopper/AfterScheduler");
  }

  public enum Goal {
    LAUNCH,
    OUTTAKE,
    STOP
  }

  public enum HopperLevel {
    /** No fuel */
    EMPTY,

    /** 0-25% full */
    FULL_0_25,

    /** 25-50% full */
    FULL_25_50,

    /** 50-75% full */
    FULL_50_75,

    /** 75-100% full */
    FULL_75_100
  }
}
