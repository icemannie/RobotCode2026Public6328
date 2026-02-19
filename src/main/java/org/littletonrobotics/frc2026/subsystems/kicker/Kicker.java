// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.kicker;

import edu.wpi.first.wpilibj.Timer;
import java.util.Optional;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.RobotContainer.SimFuelCount;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystem;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.LoggedTracer;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;

public class Kicker extends FullSubsystem {
  private static final LoggedTunableNumber rollerIntakeSpeed =
      new LoggedTunableNumber("Kicker/Roller/IntakeSetpointSpeed", 180);
  private static final LoggedTunableNumber rollerOuttakeSetpoint =
      new LoggedTunableNumber("Kicker/Roller/OuttakeSetpontSpeed", -200);

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Kicker/kP", 0.4);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Kicker/kD", 0.0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Kicker/kS", 0.3);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Kicker/kV", 0.0185);

  private final RollerSystem roller;
  private final Optional<SimFuelCount> simFuelCount;

  @Getter @Setter @AutoLogOutput private Goal goal = Goal.STOP;
  private Goal lastGoal = Goal.STOP;

  private Timer launchTimer = new Timer();

  public Kicker(RollerSystemIO rollerIO, Optional<SimFuelCount> simFuelCount) {
    this.roller = new RollerSystem("Kicker roller", "Kicker", rollerIO);
    this.simFuelCount = simFuelCount;
  }

  public void periodic() {
    roller.periodic();

    if (kP.hasChanged(hashCode()) || kD.hasChanged(hashCode())) {
      roller.setGains(kP.get(), kD.get());
    }
    if (kS.hasChanged(hashCode()) || kV.hasChanged(hashCode())) {
      roller.setFeedforward(kS.get(), kV.get());
    }

    switch (goal) {
      case LAUNCH -> {
        roller.runClosedLoop(rollerIntakeSpeed.get());
      }
      case OUTTAKE -> {
        roller.runClosedLoop(rollerOuttakeSetpoint.get());
      }
      case STOP -> {
        roller.runOpenLoop(0.0);
      }
    }

    if (Constants.getMode() == Constants.Mode.SIM
        && goal == Goal.LAUNCH
        && simFuelCount.isPresent()) {
      if (lastGoal != Goal.LAUNCH) {
        launchTimer.restart();
      } else if (launchTimer.advanceIfElapsed(1.0 / SimFuelCount.getLaunchBPS())
          && simFuelCount.get().getFuelStored() > 0) {
        simFuelCount.get().setFuelStored(Math.max(0, simFuelCount.get().getFuelStored() - 1));
      }
    }

    lastGoal = goal;

    LoggedTracer.record("Kicker/Periodic");
  }

  @Override
  public void periodicAfterScheduler() {
    roller.periodicAfterScheduler();
    LoggedTracer.record("Kicker/AfterScheduler");
  }

  public enum Goal {
    LAUNCH,
    OUTTAKE,
    STOP
  }
}
