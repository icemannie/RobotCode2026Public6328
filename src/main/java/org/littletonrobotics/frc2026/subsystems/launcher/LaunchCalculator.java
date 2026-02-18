// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.launcher;

import static org.littletonrobotics.frc2026.subsystems.launcher.LauncherConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.FieldConstants;
import org.littletonrobotics.frc2026.RobotState;
import org.littletonrobotics.frc2026.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.frc2026.util.geometry.Bounds;
import org.littletonrobotics.frc2026.util.geometry.GeomUtil;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class LaunchCalculator {
  private static LaunchCalculator instance;

  private final LinearFilter hoodAngleFilter =
      LinearFilter.movingAverage((int) (0.1 / Constants.loopPeriodSecs));
  private final LinearFilter driveAngleFilter =
      LinearFilter.movingAverage((int) (0.8 / Constants.loopPeriodSecs));

  private double lastHoodAngle;
  private Rotation2d lastDriveAngle;

  public static LaunchCalculator getInstance() {
    if (instance == null) instance = new LaunchCalculator();
    return instance;
  }

  public record LaunchingParameters(
      boolean isValid,
      Rotation2d driveAngle,
      double driveVelocity,
      double hoodAngle,
      double hoodVelocity,
      double flywheelSpeed,
      double distance,
      double distanceNoLookahead,
      double timeOfFlight,
      boolean passing) {}

  // Cache parameters
  private LaunchingParameters latestParameters = null;

  private static final double minDistance;
  private static final double maxDistance;
  private static final double passingMinDistance;
  private static final double passingMaxDistance;
  private static final double phaseDelay;

  // Launching Maps
  private static final InterpolatingTreeMap<Double, Rotation2d> hoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap flywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap timeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Passing Maps
  private static final InterpolatingTreeMap<Double, Rotation2d> passingHoodAngleMap =
      new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
  private static final InterpolatingDoubleTreeMap passingFlywheelSpeedMap =
      new InterpolatingDoubleTreeMap();
  private static final InterpolatingDoubleTreeMap passingTimeOfFlightMap =
      new InterpolatingDoubleTreeMap();

  // Passing targets
  private static final double hubPassLine =
      FieldConstants.LinesHorizontal.rightBumpStart - DriveConstants.trackWidthY / 2.0;
  private static final double xPassTarget = Units.inchesToMeters(25);
  private static final double yPassTarget = Units.inchesToMeters(50);
  // Boxes of bad
  // Under tower
  private static final Bounds towerBound =
      new Bounds(0, Units.inchesToMeters(46), Units.inchesToMeters(129), Units.inchesToMeters(168));

  // Behind the hubs
  private static final Bounds nearHubBound =
      new Bounds(
          FieldConstants.LinesVertical.neutralZoneNear,
          FieldConstants.LinesVertical.neutralZoneNear + Units.inchesToMeters(65),
          FieldConstants.LinesHorizontal.rightBumpStart,
          FieldConstants.LinesHorizontal.leftBumpEnd);
  private static final Bounds farHubBound =
      new Bounds(
          FieldConstants.LinesVertical.oppAllianceZone,
          FieldConstants.fieldLength,
          FieldConstants.LinesHorizontal.rightBumpStart,
          FieldConstants.LinesHorizontal.leftBumpEnd);

  static {
    minDistance = 1.34;
    maxDistance = 5.60;
    // TODO: define actual values when we tune the map
    passingMinDistance = 0.0;
    passingMaxDistance = 100000;
    phaseDelay = 0.03;

    hoodAngleMap.put(1.34, Rotation2d.fromDegrees(19.0));
    hoodAngleMap.put(1.78, Rotation2d.fromDegrees(19.0));
    hoodAngleMap.put(2.17, Rotation2d.fromDegrees(24.0));
    hoodAngleMap.put(2.81, Rotation2d.fromDegrees(27.0));
    hoodAngleMap.put(3.82, Rotation2d.fromDegrees(29.0));
    hoodAngleMap.put(4.09, Rotation2d.fromDegrees(30.0));
    hoodAngleMap.put(4.40, Rotation2d.fromDegrees(31.0));
    hoodAngleMap.put(4.77, Rotation2d.fromDegrees(32.0));
    hoodAngleMap.put(5.57, Rotation2d.fromDegrees(32.0));
    hoodAngleMap.put(5.60, Rotation2d.fromDegrees(35.0));

    flywheelSpeedMap.put(1.34, 210.0);
    flywheelSpeedMap.put(1.78, 220.0);
    flywheelSpeedMap.put(2.17, 220.0);
    flywheelSpeedMap.put(2.81, 230.0);
    flywheelSpeedMap.put(3.82, 250.0);
    flywheelSpeedMap.put(4.09, 255.0);
    flywheelSpeedMap.put(4.40, 260.0);
    flywheelSpeedMap.put(4.77, 265.0);
    flywheelSpeedMap.put(5.57, 275.0);
    flywheelSpeedMap.put(5.60, 290.0);

    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);

    passingHoodAngleMap.put(passingMinDistance, Rotation2d.fromDegrees(0.0));
    passingHoodAngleMap.put(passingMaxDistance, Rotation2d.fromDegrees(0.0));

    passingFlywheelSpeedMap.put(passingMinDistance, 0.0);
    passingFlywheelSpeedMap.put(passingMaxDistance, 0.0);

    passingTimeOfFlightMap.put(passingMinDistance, 0.0);
    passingTimeOfFlightMap.put(passingMaxDistance, 0.0);
  }

  public static double getMinTimeOfFlight() {
    return timeOfFlightMap.get(minDistance);
  }

  public static double getMaxTimeOfFlight() {
    return timeOfFlightMap.get(maxDistance);
  }

  public LaunchingParameters getParameters() {
    boolean passing =
        AllianceFlipUtil.applyX(RobotState.getInstance().getEstimatedPose().getX())
            > FieldConstants.LinesVertical.hubCenter;
    if (latestParameters != null) {
      return latestParameters;
    }

    // Calculate estimated pose while accounting for phase delay
    Pose2d estimatedPose = RobotState.getInstance().getEstimatedPose();
    ChassisSpeeds robotRelativeVelocity = RobotState.getInstance().getRobotVelocity();
    estimatedPose =
        estimatedPose.exp(
            new Twist2d(
                robotRelativeVelocity.vxMetersPerSecond * phaseDelay,
                robotRelativeVelocity.vyMetersPerSecond * phaseDelay,
                robotRelativeVelocity.omegaRadiansPerSecond * phaseDelay));

    // Calculate target
    Translation2d target =
        AllianceFlipUtil.apply(
            passing ? getPassingTarget() : FieldConstants.Hub.topCenterPoint.toTranslation2d());
    Pose2d launcherPosition = estimatedPose.transformBy(robotToLauncher.toTransform2d());
    double launcherToTargetDistance = target.getDistance(launcherPosition.getTranslation());

    // Calculate field relative launcher velocity
    // This isn't actually the launcherVelocity given it won't account for angular velocity of robot
    double launcherVelocityX = RobotState.getInstance().getFieldVelocity().vxMetersPerSecond;
    double launcherVelocityY = RobotState.getInstance().getFieldVelocity().vyMetersPerSecond;

    // Account for imparted velocity by robot (launcher) to offset
    double timeOfFlight =
        passing
            ? passingTimeOfFlightMap.get(launcherToTargetDistance)
            : timeOfFlightMap.get(launcherToTargetDistance);
    Pose2d lookaheadPose = launcherPosition;
    double lookaheadLauncherToTargetDistance = launcherToTargetDistance;

    for (int i = 0; i < 20; i++) {
      timeOfFlight =
          passing
              ? passingTimeOfFlightMap.get(lookaheadLauncherToTargetDistance)
              : timeOfFlightMap.get(lookaheadLauncherToTargetDistance);
      double offsetX = launcherVelocityX * timeOfFlight;
      double offsetY = launcherVelocityY * timeOfFlight;
      lookaheadPose =
          new Pose2d(
              launcherPosition.getTranslation().plus(new Translation2d(offsetX, offsetY)),
              launcherPosition.getRotation());
      lookaheadLauncherToTargetDistance = target.getDistance(lookaheadPose.getTranslation());
    }

    // Account for launcher being off center
    Pose2d lookaheadRobotPose =
        lookaheadPose.transformBy(robotToLauncher.toTransform2d().inverse());
    Rotation2d driveAngle = getDriveAngleWithLauncherOffset(lookaheadRobotPose, target);

    // Calculate remaining parameters
    double hoodAngle =
        passing
            ? passingHoodAngleMap.get(lookaheadLauncherToTargetDistance).getRadians()
            : hoodAngleMap.get(lookaheadLauncherToTargetDistance).getRadians();
    if (lastDriveAngle == null) lastDriveAngle = driveAngle;
    if (Double.isNaN(lastHoodAngle)) lastHoodAngle = hoodAngle;
    double hoodVelocity =
        hoodAngleFilter.calculate((hoodAngle - lastHoodAngle) / Constants.loopPeriodSecs);
    lastHoodAngle = hoodAngle;
    double driveVelocity =
        driveAngleFilter.calculate(
            driveAngle.minus(lastDriveAngle).getRadians() / Constants.loopPeriodSecs);
    lastDriveAngle = driveAngle;

    // Check if inside a box of bad
    var flippedPose = AllianceFlipUtil.apply(estimatedPose);
    boolean insideTowerBadBox = towerBound.contains(flippedPose.getTranslation());
    boolean behindNearHub = nearHubBound.contains(flippedPose.getTranslation());
    boolean behindFarHub = farHubBound.contains(flippedPose.getTranslation());
    boolean outsideOfBadBoxes = !(insideTowerBadBox || behindNearHub || behindFarHub);

    // Constructor parameters
    latestParameters =
        new LaunchingParameters(
            outsideOfBadBoxes
                && lookaheadLauncherToTargetDistance >= (passing ? passingMinDistance : minDistance)
                && lookaheadLauncherToTargetDistance
                    <= (passing ? passingMaxDistance : maxDistance),
            driveAngle,
            driveVelocity,
            hoodAngle,
            hoodVelocity,
            passing
                ? passingFlywheelSpeedMap.get(lookaheadLauncherToTargetDistance)
                : flywheelSpeedMap.get(lookaheadLauncherToTargetDistance),
            lookaheadLauncherToTargetDistance,
            launcherToTargetDistance,
            timeOfFlight,
            passing);

    // Log calculated values
    Logger.recordOutput("LaunchCalculator/TargetPose", new Pose2d(target, Rotation2d.kZero));
    Logger.recordOutput("LaunchCalculator/LookaheadPose", lookaheadPose);
    Logger.recordOutput(
        "LaunchCalculator/LauncherToTargetDistance", lookaheadLauncherToTargetDistance);

    return latestParameters;
  }

  private static Rotation2d getDriveAngleWithLauncherOffset(
      Pose2d robotPose, Translation2d target) {
    Rotation2d fieldToHubAngle = target.minus(robotPose.getTranslation()).getAngle();
    Rotation2d hubAngle =
        new Rotation2d(
            Math.asin(
                MathUtil.clamp(
                    robotToLauncher.getTranslation().getY()
                        / target.getDistance(robotPose.getTranslation()),
                    -1.0,
                    1.0)));
    Rotation2d driveAngle =
        fieldToHubAngle.plus(hubAngle).plus(robotToLauncher.getRotation().toRotation2d());
    return driveAngle;
  }

  public double getNaiveTOF(double distance) {
    return timeOfFlightMap.get(distance);
  }

  public void clearLaunchingParameters() {
    latestParameters = null;
  }

  public Translation2d getPassingTarget() {
    double flippedY = AllianceFlipUtil.apply(RobotState.getInstance().getEstimatedPose()).getY();
    boolean mirror = flippedY > FieldConstants.LinesHorizontal.center;

    // Check if we need to interpolate
    if (FieldConstants.fieldWidth - hubPassLine > flippedY && flippedY > hubPassLine) {
      double interpolateZoneAmount =
          ((mirror ? FieldConstants.fieldWidth - flippedY : flippedY) - hubPassLine)
              / (FieldConstants.LinesHorizontal.center - hubPassLine);
      var unflippedPoseY =
          mirror
              ? FieldConstants.fieldWidth
                  - MathUtil.interpolate(yPassTarget, passingMinDistance, interpolateZoneAmount)
              : MathUtil.interpolate(yPassTarget, passingMinDistance, interpolateZoneAmount);
      Translation2d flippedGoalTranslation =
          AllianceFlipUtil.apply(new Translation2d(xPassTarget, unflippedPoseY));
      return flippedGoalTranslation;
    }

    // Fixed passing target
    Translation2d flippedGoalTranslation =
        AllianceFlipUtil.apply(
            new Translation2d(
                xPassTarget, mirror ? FieldConstants.fieldWidth - yPassTarget : yPassTarget));

    return flippedGoalTranslation;
  }

  /**
   * Returns the Pose2d that correctly aims the robot at the goal for a given robot translation.
   *
   * @param robotTranslation The translation of the center of the robot.
   * @return The target pose for the aimed robot.
   */
  public static Pose2d getStationaryAimedPose(Translation2d robotTranslation) {
    // Calculate target
    Translation2d target =
        AllianceFlipUtil.apply(FieldConstants.Hub.topCenterPoint.toTranslation2d());

    return new Pose2d(
        robotTranslation, getDriveAngleWithLauncherOffset(robotTranslation.toPose2d(), target));
  }
}
