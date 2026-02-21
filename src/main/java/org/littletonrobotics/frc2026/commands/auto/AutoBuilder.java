// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.commands.auto;

import static org.littletonrobotics.frc2026.commands.auto.AutoCommands.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;
import lombok.RequiredArgsConstructor;
import org.littletonrobotics.frc2026.AutoFieldConstants.*;
import org.littletonrobotics.frc2026.AutoSelector.AutoQuestionResponse;
import org.littletonrobotics.frc2026.RobotState;
import org.littletonrobotics.frc2026.subsystems.drive.Drive;
import org.littletonrobotics.frc2026.subsystems.hopper.Hopper;
import org.littletonrobotics.frc2026.subsystems.kicker.Kicker;
import org.littletonrobotics.frc2026.subsystems.launcher.LaunchCalculator;
import org.littletonrobotics.frc2026.subsystems.launcher.flywheel.Flywheel;
import org.littletonrobotics.frc2026.subsystems.launcher.hood.Hood;
import org.littletonrobotics.frc2026.subsystems.slamtake.Slamtake;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;

@RequiredArgsConstructor
@SuppressWarnings("unused")
public class AutoBuilder {
  private final Drive drive;
  private final Slamtake intake;
  private final Hopper hopper;
  private final Kicker kicker;
  private final Hood hood;
  private final Flywheel flywheel;

  private final Supplier<List<AutoQuestionResponse>> responses;

  public Command homeDepotSalesman() {
    return Commands.sequence(
        Commands.either(
                followTrajectory("HomeDepot_startToOutpost", drive, true),
                followTrajectory("HomeDepot_startToOutpostAround", drive, true),
                () -> responses.get().get(0).equals(AutoQuestionResponse.YES))
            .andThen(Commands.waitSeconds(2.5)),
        AutoCommands.driveToPose(drive, () -> Launch.rightTower)
            .raceWith(
                Commands.sequence(
                    AutoCommands.waitUntilWithinTolerance(
                        Launch.rightTower, 0.1, Rotation2d.fromDegrees(5)),
                    index(hopper, kicker, flywheel, intake)
                        .withDeadline(
                            Commands.waitSeconds(
                                10)))), // TODO: Replace wait with hopper depth checks
        Commands.select(
            Map.of(
                AutoQuestionResponse.CLIMB,
                followTrajectory("launchRightTowerToClimbRight", drive, false),
                AutoQuestionResponse.NOTHING,
                Commands.none()),
            () -> responses.get().get(1)));
  }

  public Command lowesHardwareSalesman() {
    return Commands.sequence(
        // Intake from outpost
        followTrajectory("trenchRightStartToOutpost", drive, true),
        Commands.waitSeconds(2.5),

        // Intake from depot
        Commands.either(
            Commands.none(), // TODO: Implement --> followTrajectory("LowesHardware_outpostToDepot",
            // drive, false),
            followTrajectory("LowesHardware_outpostToDepotAround", drive, false),
            () -> responses.get().get(0).equals(AutoQuestionResponse.YES)),

        // Launch intaken fuel
        AutoCommands.driveToPose(drive, () -> Launch.leftTower)
            .raceWith(
                Commands.sequence(
                    AutoCommands.waitUntilWithinTolerance(
                        Launch.leftTower, 0.1, Rotation2d.fromDegrees(5)),
                    index(hopper, kicker, flywheel, intake)
                        .withDeadline(
                            Commands.waitSeconds(
                                10)))), // TODO: Replace wait with hopper depth checks
        Commands.select(
            Map.of(
                AutoQuestionResponse.CLIMB,
                followTrajectory("launchLeftTowerToClimbLeft", drive, false),
                AutoQuestionResponse.NOTHING,
                Commands.none()),
            () -> responses.get().get(1)));
  }

  public Command monopolySalesman() {
    final BooleanSupplier isLeft =
        () ->
            RobotState.getInstance()
                    .getEstimatedPose()
                    .getTranslation()
                    .getDistance(AllianceFlipUtil.apply(Launch.leftTower.getTranslation()))
                < RobotState.getInstance()
                    .getEstimatedPose()
                    .getTranslation()
                    .getDistance(AllianceFlipUtil.apply(Launch.rightTower.getTranslation()));
    return Commands.sequence(
        Commands.select(
            Map.of(
                AutoQuestionResponse.LEFT_TRENCH,
                followTrajectory("trenchLeftStartToTower", drive, true),
                AutoQuestionResponse.LEFT_BUMP,
                followTrajectory("bumpLeftInnerToTower", drive, true),
                AutoQuestionResponse.RIGHT_BUMP,
                followTrajectory("bumpRightInnerToOutpost", drive, true)
                    .andThen(Commands.waitSeconds(3)),
                AutoQuestionResponse.RIGHT_TRENCH,
                followTrajectory("trenchRightStartToOutpost", drive, true)
                    .andThen(Commands.waitSeconds(3))),
            () -> responses.get().get(0)),
        Commands.either(
                AutoCommands.driveToPose(drive, () -> Launch.leftTower),
                AutoCommands.driveToPose(drive, () -> Launch.rightTower),
                isLeft)
            .raceWith(
                Commands.sequence(
                    AutoCommands.waitUntilWithinTolerance(
                        () -> isLeft.getAsBoolean() ? Launch.leftTower : Launch.rightTower,
                        0.1,
                        Rotation2d.fromDegrees(5)),
                    index(hopper, kicker, flywheel, intake))));
  }

  public Command timidSalesman() {
    Supplier<Pose2d> target =
        () -> {
          Translation2d offset = new Translation2d();
          switch (responses.get().get(0)) {
            case CENTER:
            case LEFT_BUMP:
            case RIGHT_BUMP:
              offset = new Translation2d(-1, 0);
              break;
            case LEFT_TRENCH:
              offset = new Translation2d(-1, -0.5);
              break;
            case RIGHT_TRENCH:
              offset = new Translation2d(-1, 0.5);
              break;
            default:
              break;
          }

          return LaunchCalculator.getStationaryAimedPose(
              responses.get().get(0).getWaypoint().translation().plus(offset), true);
        };

    return Commands.sequence(
        AutoCommands.resetStartingPose(() -> responses.get().get(0)),
        Commands.parallel(
            AutoCommands.driveToPose(drive, target),
            Commands.sequence(
                waitUntilWithinTolerance(target, 0.1, Rotation2d.fromDegrees(5)),
                index(hopper, kicker, flywheel, intake))));
  }

  public Command driveForward1m() {
    return followTrajectory("DriveForward1m", drive, true);
  }

  public Command driveForward1mWhileTurn90() {
    return followTrajectory("DriveForward1mWhileTurn90", drive, true);
  }

  public Command driveForward1mWhileTurn90VelLim() {
    return followTrajectory("DriveForward1mWhileTurn90VelLim", drive, true);
  }
}
