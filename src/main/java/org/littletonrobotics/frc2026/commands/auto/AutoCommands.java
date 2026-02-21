// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.commands.auto;

import choreo.Choreo;
import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.frc2026.AutoFieldConstants;
import org.littletonrobotics.frc2026.AutoSelector.AutoQuestionResponse;
import org.littletonrobotics.frc2026.FieldConstants;
import org.littletonrobotics.frc2026.RobotState;
import org.littletonrobotics.frc2026.commands.DriveToPose;
import org.littletonrobotics.frc2026.commands.DriveTrajectory;
import org.littletonrobotics.frc2026.subsystems.drive.Drive;
import org.littletonrobotics.frc2026.subsystems.hopper.Hopper;
import org.littletonrobotics.frc2026.subsystems.kicker.Kicker;
import org.littletonrobotics.frc2026.subsystems.launcher.flywheel.Flywheel;
import org.littletonrobotics.frc2026.subsystems.slamtake.Slamtake;
import org.littletonrobotics.frc2026.subsystems.slamtake.Slamtake.SlamGoal;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;

public class AutoCommands {
  public static Command returnToLaunchPose(Drive drive) {
    return new DriveToPose(
        drive,
        () -> {
          Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
          Pose2d target =
              currentPose
                          .getTranslation()
                          .getDistance(
                              AllianceFlipUtil.apply(
                                  AutoFieldConstants.Launch.leftBump.getTranslation()))
                      < currentPose
                          .getTranslation()
                          .getDistance(
                              AllianceFlipUtil.apply(
                                  AutoFieldConstants.Launch.rightBump.getTranslation()))
                  ? AllianceFlipUtil.apply(AutoFieldConstants.Launch.leftBump)
                  : AllianceFlipUtil.apply(AutoFieldConstants.Launch.rightBump);

          double minYOffset = 0.00;
          double maxYOffset = 1.5;
          double t =
              MathUtil.clamp(
                  (Math.cbrt(
                      (Math.abs(currentPose.getY() - target.getY()) - minYOffset)
                          / (maxYOffset - minYOffset))),
                  0.0,
                  1.0);

          return new Pose2d(
              MathUtil.interpolate(
                  target.getX(),
                  AllianceFlipUtil.applyX(FieldConstants.LinesVertical.neutralZoneNear + 1.5),
                  t),
              target.getY(),
              target.getRotation());
        });
  }

  public static Command followTrajectory(String name, Drive drive, boolean start) {
    Optional<Trajectory<SwerveSample>> trajectoryOptional = Choreo.loadTrajectory(name);
    if (trajectoryOptional.isPresent()) {
      Trajectory<SwerveSample> trajectory = trajectoryOptional.get();
      return Commands.sequence(
          start
              ? AutoCommands.resetPose(
                  trajectory.getInitialSample(AllianceFlipUtil.shouldFlip()).get().getPose())
              : Commands.none(),
          new DriveTrajectory(trajectory, drive));
    } else {
      throw new RuntimeException("Choreo Trajectory Not Found: " + name);
    }
  }

  public static Command index(Hopper hopper, Kicker kicker, Flywheel flywheel, Slamtake slamtake) {
    return Commands.waitUntil(flywheel::atGoal)
        .andThen(
            Commands.startEnd(
                    () -> {
                      hopper.setGoal(Hopper.Goal.LAUNCH);
                      kicker.setGoal(Kicker.Goal.LAUNCH);
                    },
                    () -> {
                      hopper.setGoal(Hopper.Goal.STOP);
                      kicker.setGoal(Kicker.Goal.STOP);
                    },
                    hopper,
                    kicker)
                .withDeadline(
                    Commands.repeatingSequence(
                        Commands.waitSeconds(1.5),
                        Commands.runOnce(() -> slamtake.setSlamGoal(SlamGoal.RETRACT)),
                        Commands.waitSeconds(0.5),
                        Commands.runOnce(() -> slamtake.setSlamGoal(SlamGoal.DEPLOY)))));
  }

  public static boolean xCrossed(double xPosition, boolean towardsCenter) {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    if (AllianceFlipUtil.shouldFlip()) {
      if (towardsCenter) {
        return robotPose.getX() < FieldConstants.fieldLength - xPosition;
      } else {
        return robotPose.getX() > FieldConstants.fieldLength - xPosition;
      }
    } else {
      if (towardsCenter) {
        return robotPose.getX() > xPosition;
      } else {
        return robotPose.getX() < xPosition;
      }
    }
  }

  public static boolean yCrossed(double yPosition, boolean towardsLeft) {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    if (AllianceFlipUtil.shouldFlip()) {
      if (towardsLeft) {
        return robotPose.getY() < FieldConstants.fieldWidth - yPosition;
      } else {
        return robotPose.getY() > FieldConstants.fieldWidth - yPosition;
      }
    } else {
      if (towardsLeft) {
        return robotPose.getY() > yPosition;
      } else {
        return robotPose.getY() < yPosition;
      }
    }
  }

  public static boolean withinTolerance(
      Pose2d target, double translationalTolerance, Rotation2d rotationalTolerance) {
    Pose2d robotPose = RobotState.getInstance().getEstimatedPose();
    return robotPose.getTranslation().getDistance(AllianceFlipUtil.apply(target.getTranslation()))
            < translationalTolerance
        && Math.abs(
                robotPose
                    .getRotation()
                    .minus(AllianceFlipUtil.apply(target.getRotation()))
                    .getRadians())
            < rotationalTolerance.getRadians();
  }

  public static Command waitUntilXCrossed(double xPosition, boolean towardsCenter) {
    return Commands.waitUntil(() -> xCrossed(xPosition, towardsCenter));
  }

  public static Command waitUntilYCrossed(double yPosition, boolean towardsLeft) {
    return Commands.waitUntil(() -> yCrossed(yPosition, towardsLeft));
  }

  public static Command waitUntilWithinTolerance(
      Pose2d target, double translationalTolerance, Rotation2d rotationalTolerance) {
    return Commands.waitUntil(
        () -> withinTolerance(target, translationalTolerance, rotationalTolerance));
  }

  public static Command waitUntilWithinTolerance(
      Supplier<Pose2d> target, double translationalTolerance, Rotation2d rotationalTolerance) {
    return Commands.waitUntil(
        () -> withinTolerance(target.get(), translationalTolerance, rotationalTolerance));
  }

  public static Command driveToPose(Drive drive, Supplier<Pose2d> target) {
    return new DriveToPose(drive, () -> AllianceFlipUtil.apply(target.get()));
  }

  /**
   * Resets pose accounting for alliance color.
   *
   * @param pose Pose to reset to.
   */
  public static Command resetPose(Pose2d pose) {
    return Commands.runOnce(
        () -> {
          RobotState.getInstance().resetPose(AllianceFlipUtil.apply(pose));
        });
  }

  public static Command resetPose(Supplier<Pose2d> pose) {
    return Commands.runOnce(
        () -> {
          RobotState.getInstance().resetPose(AllianceFlipUtil.apply(pose.get()));
        });
  }

  public static Command resetStartingPose(Supplier<AutoQuestionResponse> startingLocation) {
    return resetPose(
        () -> new Pose2d(startingLocation.get().getWaypoint().translation(), Rotation2d.kPi));
  }
}
