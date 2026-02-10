// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.commands;

import choreo.trajectory.SwerveSample;
import choreo.trajectory.Trajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import java.util.function.Supplier;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.RobotState;
import org.littletonrobotics.frc2026.subsystems.drive.Drive;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class DriveTrajectory extends Command {
  private static final LoggedTunableNumber linearkP =
      new LoggedTunableNumber("DriveTrajectory/LinearkP", 8.0);
  private static final LoggedTunableNumber linearkD =
      new LoggedTunableNumber("DriveTrajectory/LinearkD", 0.0);
  private static final LoggedTunableNumber thetakP =
      new LoggedTunableNumber("DriveTrajectory/ThetakP", 4.0);
  private static final LoggedTunableNumber thetakD =
      new LoggedTunableNumber("DriveTrajectory/ThetakD", 0.0);

  private final Timer timer = new Timer();
  private final Trajectory<SwerveSample> trajectory;
  private final Supplier<Optional<Double>> omegaOverride;
  private final Drive drive;

  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;

  public DriveTrajectory(
      Trajectory<SwerveSample> trajectory, Supplier<Optional<Double>> omegaOverride, Drive drive) {
    this.drive = drive;
    this.trajectory = trajectory;
    this.omegaOverride = omegaOverride;
    xController = new PIDController(linearkP.get(), 0, linearkD.get(), Constants.loopPeriodSecs);
    yController = new PIDController(linearkP.get(), 0, linearkD.get(), Constants.loopPeriodSecs);
    thetaController = new PIDController(thetakP.get(), 0, thetakD.get(), Constants.loopPeriodSecs);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    addRequirements(drive);
  }

  public DriveTrajectory(Trajectory<SwerveSample> trajectory, Drive drive) {
    this(trajectory, () -> Optional.empty(), drive);
  }

  @Override
  public void initialize() {
    timer.restart();
    xController.reset();
    yController.reset();
    thetaController.reset();
  }

  @Override
  public void execute() {
    if (linearkP.hasChanged(hashCode())
        || linearkD.hasChanged(hashCode())
        || thetakP.hasChanged(hashCode())
        || thetakD.hasChanged(hashCode())) {
      xController.setPID(linearkP.get(), 0, linearkD.get());
      yController.setPID(linearkP.get(), 0, linearkD.get());
      thetaController.setPID(thetakP.get(), 0, thetakD.get());
    }

    Pose2d currentPose = RobotState.getInstance().getEstimatedPose();
    var desiredState = trajectory.sampleAt(timer.get(), AllianceFlipUtil.shouldFlip()).get();

    double xOutput =
        xController.calculate(currentPose.getX(), desiredState.getPose().getX()) + desiredState.vx;
    double yOutput =
        yController.calculate(currentPose.getY(), desiredState.getPose().getY()) + desiredState.vy;
    double thetaOutput =
        omegaOverride
            .get()
            .orElseGet(
                () ->
                    thetaController.calculate(
                            currentPose.getRotation().getRadians(),
                            desiredState.getPose().getRotation().getRadians())
                        + desiredState.omega);

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            new ChassisSpeeds(xOutput, yOutput, thetaOutput), currentPose.getRotation()));

    Logger.recordOutput("DriveTrajectory/Setpoint/Pose", desiredState.getPose());
    Logger.recordOutput(
        "DriveTrajectory/Setpoint/Speeds",
        new ChassisSpeeds(desiredState.vx, desiredState.vy, desiredState.omega));
    Logger.recordOutput(
        "DriveTrajectory/TranslationError",
        currentPose.getTranslation().getDistance(desiredState.getPose().getTranslation()));
    Logger.recordOutput(
        "DriveTrajectory/RotationError",
        currentPose.getRotation().minus(desiredState.getPose().getRotation()).getDegrees());
  }

  @Override
  @AutoLogOutput(key = "DriveTrajectory/isFinished")
  public boolean isFinished() {
    return timer.hasElapsed(trajectory.getTotalTime());
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
