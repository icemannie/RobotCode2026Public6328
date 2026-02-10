// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.HashMap;
import java.util.Map;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2026.util.geometry.GeomUtil;
import org.littletonrobotics.frc2026.util.vts.request.PathRequest;
import org.littletonrobotics.frc2026.util.vts.request.PathRequestHelpers;
import org.littletonrobotics.frc2026.util.vts.request.PathRequestSegment;
import org.littletonrobotics.frc2026.util.vts.request.PathWaypoint;

@ExtensionMethod({PathRequestHelpers.class, GeomUtil.class})
public class DriveTrajectories {
  public static final Map<String, PathRequest> paths = new HashMap<>();

  static {
    paths.put(
        "Test",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(Pose2d.kZero).build(),
                        PathWaypoint.from(new Pose2d(3.0, 2.0, Rotation2d.kPi))
                            .stopped(true)
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(PathWaypoint.from(new Pose2d(3.0, 5.0, Rotation2d.kPi)).build())
                    .straightLine()
                    .build())
            .build());

    paths.put(
        "HomeDepot",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting line
                        PathWaypoint.from(new Pose2d(3.55, 7.25, Rotation2d.fromDegrees(-90)))
                            .build(),
                        // Edge of depot
                        PathWaypoint.from(new Pose2d(0.58, 7.25, Rotation2d.fromDegrees(-95)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Before tower
                        PathWaypoint.from(new Pose2d(0.53, 4.7, Rotation2d.fromDegrees(-90)))
                            .build())
                    .maxVelocity(1.5)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Through tower
                        PathWaypoint.from(new Pose2d(0.53, 2.7, Rotation2d.fromDegrees(-90)))
                            .build(),
                        // Outpost
                        PathWaypoint.from(new Pose2d(0.53, 0.9, Rotation2d.fromDegrees(-90)))
                            .stopped(true)
                            .build(),
                        // Offset from right upright
                        PathWaypoint.from(new Pose2d(2.1, 3.2, Rotation2d.kZero)).build(),
                        // Aligned to right upright
                        PathWaypoint.from(new Pose2d(1.6, 3.2, Rotation2d.kZero)).build())
                    .build())
            .stopAtStart(true)
            .stopAtEnd(true)
            .build());
  }
}
