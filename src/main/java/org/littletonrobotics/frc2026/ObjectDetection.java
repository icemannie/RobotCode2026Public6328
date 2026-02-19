// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rectangle2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2026.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2026.subsystems.vision.VisionConstants;
import org.littletonrobotics.frc2026.util.EqualsUtil;
import org.littletonrobotics.frc2026.util.FuelSim;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.frc2026.util.geometry.GeomUtil;

@ExtensionMethod({GeomUtil.class})
public class ObjectDetection {
  private Set<FuelPoseRecord> fuelPoses = new HashSet<>();

  private static final LoggedTunableNumber fuelPersistanceTime =
      new LoggedTunableNumber("ObjectDetection/FuelPersistanceTime", 15.0);
  private static final LoggedTunableNumber allowedRoll =
      new LoggedTunableNumber("ObjectDetection/AllowedRoll", Units.degreesToRadians(5));
  private static final LoggedTunableNumber allowedPitch =
      new LoggedTunableNumber("ObjectDetection/AllowedPitch", Units.degreesToRadians(5));
  private static final LoggedTunableNumber fuelOverlap =
      new LoggedTunableNumber("ObjectDetection/FuelOverlap", FieldConstants.fuelDiameter / 2.0);
  private static final double maxPossibleFuel = 504;

  private static ObjectDetection instance;
  @Setter private static FuelSim fuelSim;

  public static ObjectDetection getInstance() {
    if (instance == null) instance = new ObjectDetection();
    return instance;
  }

  /** Clears fuel that is too old or in the robot. */
  public void clearOldFuelPoses() {
    Rectangle2d robot =
        new Rectangle2d(
            RobotState.getInstance()
                .getEstimatedPose()
                .transformBy(
                    new Translation2d(
                            ((DriveConstants.intakeFarX - DriveConstants.frameWidthX) / 2.0), 0.0)
                        .toTransform2d()),
            DriveConstants.intakeFarX,
            DriveConstants.frameWidthY);
    fuelPoses =
        fuelPoses.stream()
            .filter((x) -> Timer.getTimestamp() - x.timestamp() < fuelPersistanceTime.get())
            .filter((x) -> !robot.contains(x.translation()))
            .collect(Collectors.toSet());
  }

  /** Clears fuel that is within the FOV of the specified camera. */
  public void clearFuelInCameraFOV(double timestamp, int camera) {
    // Get field to robot
    var fieldToRobotOptional = RobotState.getInstance().getEstimatedPoseAtTimestamp(timestamp);
    if (fieldToRobotOptional.isEmpty()) {
      return;
    }
    Pose2d fieldToRobot = fieldToRobotOptional.get();

    // Get field to camera
    var robotToCameraOptional = VisionConstants.cameras[camera].poseFunction().apply(timestamp);
    if (robotToCameraOptional.isEmpty()) {
      return;
    }
    Pose3d robotToCamera = robotToCameraOptional.get();
    Pose2d fieldToCamera = fieldToRobot.transformBy(robotToCamera.toPose2d().toTransform2d());

    // Clear poses
    fuelPoses =
        fuelPoses.stream()
            .filter(
                (x) ->
                    Math.abs(
                            new Transform2d(
                                    fieldToCamera, new Pose2d(x.translation, Rotation2d.kZero))
                                .getTranslation()
                                .getAngle()
                                .getRadians())
                        > VisionConstants.cameras[camera].fovRads() / 2.0)
            .collect(Collectors.toSet());
  }

  public void addFuelTxTyObservation(FuelTxTyObservation observation) {
    Optional<Rotation3d> estimatedRotation3d =
        RobotState.getInstance().getEstimatedRotation3dAtTimestamp(observation.timestamp);
    if (estimatedRotation3d.isPresent()) {
      if (!(EqualsUtil.epsilonEquals(estimatedRotation3d.get().getX(), 0, allowedRoll.get())
          && EqualsUtil.epsilonEquals(estimatedRotation3d.get().getY(), 0, allowedPitch.get()))) {
        return;
      }
    }
    if (fuelPoses.size() >= maxPossibleFuel) {
      return;
    }

    var fieldToRobotOptional =
        RobotState.getInstance().getEstimatedPoseAtTimestamp(observation.timestamp());
    if (fieldToRobotOptional.isEmpty()) {
      return;
    }
    Pose2d fieldToRobot = fieldToRobotOptional.get();

    Pose3d robotToCamera =
        VisionConstants.cameras[observation.camera()]
            .poseFunction()
            .apply(observation.timestamp())
            .get();

    // Find midpoint of width of top tx ty
    double tx = (observation.tx()[0] + observation.tx()[1]) / 2;
    double ty = (observation.ty()[0] + observation.ty()[1]) / 2;

    // Account for camera roll
    Translation2d txyxTranslation =
        new Translation2d(Math.tan(tx), Math.tan(-ty))
            .rotateBy(new Rotation2d(-robotToCamera.getRotation().getX()));
    tx = Math.atan(txyxTranslation.getX());
    ty = -Math.atan(txyxTranslation.getY());

    // No fuel above camera
    double cameraToFuelAngle = -robotToCamera.getRotation().getY() - ty;
    if (cameraToFuelAngle >= 0) {
      return;
    }

    // Top down distance to fuel from camera
    double cameraToFuelNorm =
        (-robotToCamera.getZ() + FieldConstants.fuelDiameter)
            / Math.tan(-robotToCamera.getRotation().getY() - ty)
            / Math.cos(tx);

    Pose2d fieldToCamera = fieldToRobot.transformBy(robotToCamera.toPose2d().toTransform2d());
    Pose2d fieldToFuel =
        fieldToCamera
            .transformBy(new Transform2d(Translation2d.kZero, new Rotation2d(-tx)))
            .transformBy(new Transform2d(new Translation2d(cameraToFuelNorm, 0), Rotation2d.kZero));

    Translation2d fieldToFuelTranslation2d = fieldToFuel.getTranslation();

    FuelPoseRecord fuelPoseRecord =
        new FuelPoseRecord(fieldToFuelTranslation2d, observation.timestamp());

    fuelPoses =
        fuelPoses.stream()
            .filter((x) -> x.translation.getDistance(fieldToFuelTranslation2d) > fuelOverlap.get())
            .collect(Collectors.toSet());

    fuelPoses.add(fuelPoseRecord);
  }

  public Set<Translation2d> getFuelTranslations() {
    if (Constants.getMode() == Constants.Mode.SIM) {
      return fuelSim.getFuels();
    }
    return fuelPoses.stream().map(FuelPoseRecord::translation).collect(Collectors.toSet());
  }

  public record FuelTxTyObservation(int camera, double[] tx, double[] ty, double timestamp) {}

  public record FuelPoseRecord(Translation2d translation, double timestamp) {}
}
