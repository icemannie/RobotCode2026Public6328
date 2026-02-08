// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.kicker;

import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystem;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.LoggedTracer;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;

public class Kicker extends FullSubsystem {
  private static final LoggedTunableNumber rollerIntakeFrontSpeed =
      new LoggedTunableNumber("Kicker/Roller/IntakeFrontSetpointSpeed", 180);
  private static final LoggedTunableNumber rollerIntakeBackSpeed =
      new LoggedTunableNumber("Kicker/Roller/IntakeBackSetpointSpeed", 200);
  private static final LoggedTunableNumber rollerOuttakeSetpoint =
      new LoggedTunableNumber("Kicker/Roller/OuttakeSetpontSpeed", -200);

  private static final LoggedTunableNumber frontKp =
      new LoggedTunableNumber("Kicker/RollerFront/kP", 0.4);
  private static final LoggedTunableNumber frontKd =
      new LoggedTunableNumber("Kicker/RollerFront/kD", 0.0);
  private static final LoggedTunableNumber frontKs =
      new LoggedTunableNumber("Kicker/RollerFront/kS", 0.3);
  private static final LoggedTunableNumber frontKv =
      new LoggedTunableNumber("Kicker/RollerFront/kV", 0.0185);

  private static final LoggedTunableNumber backKp =
      new LoggedTunableNumber("Kicker/RollerBack/kP", 0.5);
  private static final LoggedTunableNumber backKd =
      new LoggedTunableNumber("Kicker/RollerBack/kD", 0.0);
  private static final LoggedTunableNumber backKs =
      new LoggedTunableNumber("Kicker/RollerBack/kS", 0.5);
  private static final LoggedTunableNumber backKv =
      new LoggedTunableNumber("Kicker/RollerBack/kV", 0.022);

  private final RollerSystem rollerFront;
  private final RollerSystem rollerBack;

  @Getter @Setter @AutoLogOutput private Goal goal = Goal.STOP;

  public Kicker(RollerSystemIO rollerIOFront, RollerSystemIO rollerIOBack) {
    this.rollerFront =
        new RollerSystem(
            "Kicker roller front",
            "Kicker/RollerFront",
            rollerIOFront,
            frontKp.get(),
            frontKd.get());
    this.rollerBack =
        new RollerSystem(
            "Kicker roller back", "Kicker/RollerBack", rollerIOBack, backKp.get(), backKd.get());
  }

  public void periodic() {
    rollerFront.periodic();
    rollerBack.periodic();

    if (frontKp.hasChanged(hashCode()) || frontKd.hasChanged(hashCode())) {
      rollerFront.setGains(frontKp.get(), frontKd.get());
    }
    if (backKp.hasChanged(hashCode()) || backKd.hasChanged(hashCode())) {
      rollerBack.setGains(backKp.get(), backKd.get());
    }
    if (frontKs.hasChanged(hashCode()) || frontKv.hasChanged(hashCode())) {
      rollerFront.setFeedforward(frontKs.get(), frontKv.get());
    }
    if (backKs.hasChanged(hashCode()) || backKv.hasChanged(hashCode())) {
      rollerBack.setFeedforward(backKs.get(), backKv.get());
    }

    switch (goal) {
      case LAUNCH -> {
        rollerFront.runClosedLoop(rollerIntakeFrontSpeed.get());
        rollerBack.runClosedLoop(rollerIntakeBackSpeed.get());
      }

      case OUTTAKE -> {
        rollerFront.runClosedLoop(rollerOuttakeSetpoint.get());
        rollerBack.runClosedLoop(rollerOuttakeSetpoint.get());
      }
      case STOP -> {
        rollerFront.runOpenLoop(0.0);
        rollerBack.runOpenLoop(0.0);
      }
    }
    LoggedTracer.record("Kicker/Periodic");
  }

  @Override
  public void periodicAfterScheduler() {
    rollerFront.periodicAfterScheduler();
    rollerBack.periodicAfterScheduler();
    LoggedTracer.record("Kicker/AfterScheduler");
  }

  public enum Goal {
    LAUNCH,
    OUTTAKE,
    STOP
  }
}
