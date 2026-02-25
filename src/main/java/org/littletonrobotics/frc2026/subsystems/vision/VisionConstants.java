// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.vision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import java.util.Optional;
import java.util.function.Function;
import lombok.Builder;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2026.Constants;
import org.littletonrobotics.frc2026.util.geometry.GeomUtil;

@ExtensionMethod({GeomUtil.class})
public class VisionConstants {
  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double xyStdDevCoefficient = 0.01;
  public static final double thetaStdDevCoefficient = 0.03;

  public static final double fuelDetectConfidenceThreshold = 0.2;

  private static int monoExposure = 1800;
  private static double monoGain = 15.0;
  private static double monoDenoise = 1.0;
  private static int colorExposure = 4500;
  private static double colorGain = 5.0;

  public static CameraConfig[] cameras =
      switch (Constants.robot) {
        case COMPBOT ->
            new CameraConfig[] {
              CameraConfig.builder()
                  .poseFunction(
                      (Double timestamp) -> {
                        return Optional.of(
                            new Pose3d(
                                Units.inchesToMeters(-10.343),
                                Units.inchesToMeters(-8.102),
                                Units.inchesToMeters(20.940),
                                new Rotation3d(
                                    0.0,
                                    Units.degreesToRadians(-22.5),
                                    Units.degreesToRadians(175.0))));
                      })
                  .id("40530395")
                  .width(1800)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .denoise(monoDenoise)
                  .stdDevFactor(1.0)
                  .fovRads(Units.degreesToRadians(75.0))
                  .build()
            };
        case ALPHABOT ->
            new CameraConfig[] {
              CameraConfig.builder()
                  .poseFunction(
                      (Double timestamp) -> {
                        return Optional.of(
                            new Pose3d(
                                Units.inchesToMeters(-28.0 / 2.0 + 2.5),
                                Units.inchesToMeters(-28.0 / 2.0 + 2.75),
                                Units.inchesToMeters(18.75),
                                new Rotation3d(
                                    0.0,
                                    Units.degreesToRadians(-27.0),
                                    Units.degreesToRadians(-152.5))));
                      })
                  .id("40530395")
                  .width(1800)
                  .height(1200)
                  .exposure(monoExposure)
                  .gain(monoGain)
                  .denoise(monoDenoise)
                  .stdDevFactor(1.0)
                  .fovRads(Units.degreesToRadians(75.0))
                  .build(),
              CameraConfig.builder()
                  .poseFunction(
                      (Double timestamp) -> {
                        return Optional.of(
                            new Pose3d(
                                Units.inchesToMeters(-28.0 / 2.0 + 2.25),
                                Units.inchesToMeters(-28.0 / 2.0 + 2.75),
                                Units.inchesToMeters(18.0),
                                new Rotation3d(
                                    Units.degreesToRadians(11.0),
                                    Units.degreesToRadians(15.0),
                                    Units.degreesToRadians(-167.0))));
                      })
                  .id("24737133")
                  .width(1280)
                  .height(960)
                  .exposure(colorExposure)
                  .gain(colorGain)
                  .denoise(monoDenoise)
                  .stdDevFactor(1.0)
                  .fovRads(Units.degreesToRadians(90.0))
                  .build()
            };
        default -> new CameraConfig[] {};
      };

  @Builder
  public record CameraConfig(
      Function<Double, Optional<Pose3d>> poseFunction,
      String id,
      int width,
      int height,
      int autoExposure,
      int exposure,
      double gain,
      double denoise,
      double stdDevFactor,
      double fovRads) {}

  private VisionConstants() {}
}
