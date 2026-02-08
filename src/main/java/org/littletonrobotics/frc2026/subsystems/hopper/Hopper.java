// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.hopper;

import lombok.Getter;
import lombok.Setter;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystem;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.LoggedTracer;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;

public class Hopper extends FullSubsystem {
  private static final LoggedTunableNumber rollerLaunchVolts =
      new LoggedTunableNumber("Hopper/Roller/LaunchVolts", 12.0);
  private static final LoggedTunableNumber rollerOuttakeVolts =
      new LoggedTunableNumber("Hopper/Roller/OuttakeVolts", -12.0);
  private static final LoggedTunableNumber leftIndexerLaunchVolts =
      new LoggedTunableNumber("Hopper/Indexer/LeftLaunchVolts", 6.0);
  private static final LoggedTunableNumber leftIndexerOuttakeVolts =
      new LoggedTunableNumber("Hopper/Indexer/LeftOuttakeVolts", -6.0);
  private static final LoggedTunableNumber rightIndexerLaunchVolts =
      new LoggedTunableNumber("Hopper/Indexer/RightLaunchVolts", 6.0);
  private static final LoggedTunableNumber rightIndexerOuttakeVolts =
      new LoggedTunableNumber("Hopper/Indexer/LeftOuttakeVolts", -6.0);

  private final RollerSystem roller;
  private final RollerSystem rollerRight;

  @Getter @Setter @AutoLogOutput private Goal goal = Goal.STOP;

  public Hopper(RollerSystemIO rollerIO, RollerSystemIO rollerIORight) {
    this.roller = new RollerSystem("Hopper roller left", "Hopper/RollerLeft", rollerIO);
    this.rollerRight = new RollerSystem("Hopper roller right", "Hopper/RollerRight", rollerIORight);
  }

  public void periodic() {

    roller.periodic();
    rollerRight.periodic();

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
    rollerRight.runOpenLoop(rollerVolts);
    LoggedTracer.record("Hopper/Periodic");
  }

  @Override
  public void periodicAfterScheduler() {
    roller.periodicAfterScheduler();
    rollerRight.periodicAfterScheduler();
    LoggedTracer.record("Hopper/AfterScheduler");
  }

  public enum Goal {
    LAUNCH,
    OUTTAKE,
    STOP
  }
}
