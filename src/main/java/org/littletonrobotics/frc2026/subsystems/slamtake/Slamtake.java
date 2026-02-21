// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.slamtake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;
import java.util.function.BooleanSupplier;
import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2026.RobotState;
import org.littletonrobotics.frc2026.RobotState.SlamtakeObservation;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystem;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2026.subsystems.slamtake.SlamIO.SlamIOOutputMode;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.LoggedTracer;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;

public class Slamtake extends FullSubsystem {
  private static final LoggedTunableNumber rollerIntakeVolts =
      new LoggedTunableNumber("Slamtake/Roller/IntakeVolts", 10.0);
  private static final LoggedTunableNumber rollerOuttakeVolts =
      new LoggedTunableNumber("Slamtake/Roller/OuttakeVolts", -6.0);
  private static final LoggedTunableNumber deployAmps =
      new LoggedTunableNumber("Slamtake/Slam/DeployAmps", -25.0);
  private static final LoggedTunableNumber retractAmps =
      new LoggedTunableNumber("Slamtake/Slam/RetractAmps", 50.0);
  private final RollerSystem roller;
  private final Slam slam;

  private static final LoggedTunableNumber slamVelocityDebounceTime =
      new LoggedTunableNumber("Slamtake/Slam/DebounceTime", 0.5);
  private Debouncer slamVelocityDebouncer =
      new Debouncer(slamVelocityDebounceTime.get(), DebounceType.kRising);
  private static final LoggedTunableNumber slamSettleVelocity =
      new LoggedTunableNumber("Slamtake/Slam/SettleVelocity", 0.1);

  @Getter @Setter @AutoLogOutput private IntakeGoal intakeGoal = IntakeGoal.STOP;
  @Getter @Setter @AutoLogOutput private SlamGoal slamGoal = SlamGoal.RETRACT;
  @Getter @AutoLogOutput private SlamState slamState = SlamState.MOVING;

  public Slamtake(SlamIO slamIO, RollerSystemIO rollerIO) {
    this.slam = new Slam(slamIO);
    this.roller = new RollerSystem("Intake roller", "Slamtake/Roller", rollerIO);
  }

  public void periodic() {
    slam.periodic();
    roller.periodic();

    if (slamVelocityDebounceTime.hasChanged(hashCode())) {
      slamVelocityDebouncer = new Debouncer(slamVelocityDebounceTime.get());
    }

    double rollerVolts = 0.0;
    switch (intakeGoal) {
      case INTAKE -> {
        rollerVolts = rollerIntakeVolts.get();
      }

      case OUTTAKE -> {
        rollerVolts = rollerOuttakeVolts.get();
      }
      case STOP -> {
        rollerVolts = 0.0;
      }
    }
    roller.runOpenLoop(rollerVolts);
    switch (slamGoal) {
      case DEPLOY -> {
        slam.runAmps(deployAmps.get());
        slamState =
            slamVelocityDebouncer.calculate(Math.abs(slam.getVelocity()) < slamSettleVelocity.get())
                ? SlamState.DEPLOYED
                : SlamState.MOVING;
      }
      case RETRACT -> {
        slam.runAmps(retractAmps.get());
        slamState =
            slamVelocityDebouncer.calculate(Math.abs(slam.getVelocity()) < slamSettleVelocity.get())
                ? SlamState.RETRACTED
                : SlamState.MOVING;
      }
      case IDLE -> {
        slam.setMode(SlamIOOutputMode.COAST);
        slamState = SlamState.MOVING;
      }
    }

    // Send slamtake data to RobotState
    RobotState.getInstance()
        .addSlamtakeObservation(new SlamtakeObservation(Timer.getFPGATimestamp(), slamState));

    // Record cycle time
    LoggedTracer.record("Slamtake/Periodic");
  }

  @Override
  public void periodicAfterScheduler() {
    roller.periodicAfterScheduler();
    slam.periodicAfterScheduler();

    LoggedTracer.record("Slamtake/AfterScheduler");
  }

  public void setCoastOverride(BooleanSupplier coast) {
    slam.setCoastOverride(coast);
    roller.setCoastOverride(coast);
  }

  public enum IntakeGoal {
    INTAKE,
    OUTTAKE,
    STOP
  }

  public enum SlamGoal {
    DEPLOY,
    RETRACT,
    IDLE
  }

  public enum SlamState {
    DEPLOYED,
    RETRACTED,
    MOVING
  }
}
