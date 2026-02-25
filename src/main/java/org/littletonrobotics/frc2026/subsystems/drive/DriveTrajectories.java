// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import java.util.HashMap;
import java.util.Map;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2026.AutoFieldConstants.*;
import org.littletonrobotics.frc2026.FieldConstants;
import org.littletonrobotics.frc2026.util.geometry.GeomUtil;
import org.littletonrobotics.frc2026.util.vts.request.PathRequest;
import org.littletonrobotics.frc2026.util.vts.request.PathRequest.PathRequestBuilder;
import org.littletonrobotics.frc2026.util.vts.request.PathRequestHelpers;
import org.littletonrobotics.frc2026.util.vts.request.PathRequestSegment;
import org.littletonrobotics.frc2026.util.vts.request.PathRequestSegment.PointTarget;
import org.littletonrobotics.frc2026.util.vts.request.PathWaypoint;

@ExtensionMethod({PathRequestHelpers.class, GeomUtil.class})
public class DriveTrajectories {
  public static final Map<String, PathRequest> paths = new HashMap<>();

  static {
    paths.put(
        // Tests
        "Example",
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
        "DriveForward1m",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(Pose2d.kZero).build(),
                        PathWaypoint.from(new Pose2d(1.0, 0.0, Rotation2d.kZero))
                            .stopped(true)
                            .build())
                    .build())
            .build());

    paths.put(
        "DriveForward1mWhileTurn90",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(Pose2d.kZero).build(),
                        PathWaypoint.from(new Pose2d(1.0, 0.0, Rotation2d.kCW_90deg))
                            .stopped(true)
                            .build())
                    .build())
            .build());

    paths.put(
        "DriveForward1mWhileTurn90VelLim",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(Pose2d.kZero).build(),
                        PathWaypoint.from(new Pose2d(1.0, 0.0, Rotation2d.kCW_90deg))
                            .stopped(true)
                            .build())
                    .maxVelocity(0.5)
                    .build())
            .build());

    // Launch --> Neutral Zone
    paths.put(
        "launchLeftBumpToNeutralZone",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(Launch.leftBump).build(),
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.starting
                                        - DriveConstants.fullWidthX / 2.0,
                                    FieldConstants.LinesHorizontal.leftBumpMiddle,
                                    Rotation2d.fromDegrees(45)))
                            .build(),
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.neutralZoneNear
                                        + DriveConstants.fullWidthX / 2.0,
                                    FieldConstants.LinesHorizontal.leftBumpMiddle,
                                    Rotation2d.fromDegrees(45)))
                            .build(),
                        PathWaypoint.from(
                                new Translation2d(
                                    FieldConstants.LinesVertical.neutralZoneNear + 2.0,
                                    FieldConstants.LinesHorizontal.leftBumpMiddle))
                            .build())
                    .build())
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchRightBumpToNeutralZone",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(Launch.rightBump).build(),
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.starting
                                        - DriveConstants.fullWidthX / 2.0,
                                    FieldConstants.LinesHorizontal.rightBumpMiddle,
                                    Rotation2d.fromDegrees(-45)))
                            .build(),
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.neutralZoneNear
                                        + DriveConstants.fullWidthX / 2.0,
                                    FieldConstants.LinesHorizontal.rightBumpMiddle,
                                    Rotation2d.fromDegrees(-45)))
                            .build(),
                        PathWaypoint.from(
                                new Translation2d(
                                    FieldConstants.LinesVertical.neutralZoneNear + 2.0,
                                    FieldConstants.LinesHorizontal.rightBumpMiddle))
                            .build())
                    .build())
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchLeftTowerToNeutralZone",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(Launch.leftTower).build(),
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.starting
                                        - DriveConstants.fullWidthX / 2.0,
                                    FieldConstants.LinesHorizontal.leftBumpMiddle,
                                    Rotation2d.fromDegrees(45)))
                            .build(),
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.neutralZoneNear
                                        + DriveConstants.fullWidthX / 2.0,
                                    FieldConstants.LinesHorizontal.leftBumpMiddle,
                                    Rotation2d.fromDegrees(45)))
                            .build(),
                        PathWaypoint.from(
                                new Translation2d(
                                    FieldConstants.LinesVertical.neutralZoneNear + 2.0,
                                    FieldConstants.LinesHorizontal.leftBumpMiddle))
                            .build())
                    .build())
            .stopAtEnd(false)
            .build());

    paths.put(
        "launchRightTowerToNeutralZone",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(Launch.rightTower).build(),
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.starting
                                        - DriveConstants.fullWidthX / 2.0,
                                    FieldConstants.LinesHorizontal.rightBumpMiddle,
                                    Rotation2d.fromDegrees(-45)))
                            .build(),
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.LinesVertical.neutralZoneNear
                                        + DriveConstants.fullWidthX / 2.0,
                                    FieldConstants.LinesHorizontal.rightBumpMiddle,
                                    Rotation2d.fromDegrees(-45)))
                            .build(),
                        PathWaypoint.from(
                                new Translation2d(
                                    FieldConstants.LinesVertical.neutralZoneNear + 2.0,
                                    FieldConstants.LinesHorizontal.rightBumpMiddle))
                            .build())
                    .build())
            .stopAtEnd(false)
            .build());

    // Home Depot Salesman trajectories
    paths.put(
        "launchRightTowerToClimbRight",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(Launch.rightTower).build(),
                        PathWaypoint.from(Climb.rightOffset).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(PathWaypoint.from(Climb.right).build())
                    .keepInLaneWidth(0.008)
                    .maxVelocity(0.5)
                    .maxAngularVelocity(0.01)
                    .build())
            .build());

    PathRequestBuilder trenchLeftStartToTower =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting line
                        PathWaypoint.from(
                                new Pose2d(
                                    Trench.leftStart.translation(), Rotation2d.fromDegrees(-90)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Edge of depot
                        PathWaypoint.from(
                                new Pose2d(
                                    Depot.leftThrough.translation(), Rotation2d.fromDegrees(-105)))
                            .build())
                    .keepInLaneWidth(0.3)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Through Depot
                        PathWaypoint.from(
                                new Pose2d(
                                    Depot.rightThrough.translation(), Rotation2d.fromDegrees(-105)))
                            .build())
                    .maxVelocity(1.5)
                    .maxAngularVelocity(0.1)
                    .keepInLaneWidth(0.05)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Before Tower Intermediate
                        PathWaypoint.from(new Translation2d(0.75, 5.0)).build())
                    .build());

    PathRequestBuilder bumpLeftInnerToTower =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting line
                        PathWaypoint.from(
                                new Pose2d(
                                    Bump.leftInner.translation(), Rotation2d.fromDegrees(-90)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Edge of depot
                        PathWaypoint.from(
                                new Pose2d(
                                    Depot.leftThrough.translation(), Rotation2d.fromDegrees(-105)))
                            .build())
                    .keepInLaneWidth(0.3)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Through Depot
                        PathWaypoint.from(
                                new Pose2d(
                                    Depot.rightThrough.translation(), Rotation2d.fromDegrees(-105)))
                            .build())
                    .maxVelocity(1.5)
                    .maxAngularVelocity(0.1)
                    .keepInLaneWidth(0.05)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Before Tower Intermediate
                        PathWaypoint.from(new Translation2d(0.75, 5.0)).build())
                    .build());

    paths.put("trenchLeftStartToTower", trenchLeftStartToTower.build());

    paths.put("bumpLeftInnerToTower", bumpLeftInnerToTower.build());

    paths.put(
        "HomeDepot_startToOutpost",
        trenchLeftStartToTower
            .build()
            .clone()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Before tower
                        PathWaypoint.from(new Pose2d(0.53, 4.68, Rotation2d.fromDegrees(-90)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Through tower
                        PathWaypoint.from(new Pose2d(0.53, 2.7, Rotation2d.fromDegrees(-90)))
                            .build(),
                        // Outpost
                        PathWaypoint.from(new Pose2d(0.53, 0.9, Rotation2d.fromDegrees(-90)))
                            .build())
                    .maxVelocity(1.5)
                    .maxAngularVelocity(0.1)
                    .build())
            .build());

    paths.put(
        "HomeDepot_startToOutpostAround",
        trenchLeftStartToTower
            .build()
            .clone()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Outside tower left
                        PathWaypoint.from(Tower.leftOutside.translation()).build())
                    .keepInLaneWidth(0.15)
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Outside tower right
                        PathWaypoint.from(Tower.rightOutside.translation()).build())
                    .pointAt(
                        new PointTarget(
                            FieldConstants.Hub.innerCenterPoint.toTranslation2d(),
                            Rotation2d.fromDegrees(10),
                            true))
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Outpost
                        PathWaypoint.from(
                                new Pose2d(
                                    FieldConstants.Outpost.centerPoint.plus(
                                        new Translation2d(
                                            DriveConstants.fullWidthX / 2.0 + 0.25, 0.0)),
                                    Rotation2d.kPi))
                            .build())
                    .build())
            .build());

    // Lowe's Hardware Salesman trajectories
    Pose2d LH_outpost =
        new Pose2d(
            FieldConstants.Outpost.centerPoint.plus(
                new Translation2d(DriveConstants.fullWidthX / 2.0 + 0.25, 0.15)),
            Rotation2d.kPi);

    PathRequestBuilder trenchRightStartToOutpost =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting line
                        PathWaypoint.from(
                                new Pose2d(Trench.rightStart.translation(), Rotation2d.kPi))
                            .build(),
                        // Outpost
                        PathWaypoint.from(LH_outpost).build())
                    .keepInLaneWidth(0.15)
                    .build());

    PathRequestBuilder bumpRightInnerToOutpost =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Starting line
                        PathWaypoint.from(new Pose2d(Bump.rightInner.translation(), Rotation2d.kPi))
                            .build(),
                        // Outpost
                        PathWaypoint.from(LH_outpost).build())
                    .keepInLaneWidth(0.15)
                    .build());

    PathRequestBuilder depotRightToLeft =
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Edge of depot
                        PathWaypoint.from(
                                new Pose2d(
                                    Depot.rightThrough.translation(), Rotation2d.fromDegrees(105)))
                            .build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Through Depot
                        PathWaypoint.from(
                                new Pose2d(
                                    Depot.leftThrough.translation(), Rotation2d.fromDegrees(105)))
                            .build())
                    .maxVelocity(1.5)
                    .maxAngularVelocity(0.1)
                    .keepInLaneWidth(0.05)
                    .build());

    paths.put("trenchRightStartToOutpost", trenchRightStartToOutpost.build());

    paths.put("bumpRightInnerToOutpost", bumpRightInnerToOutpost.build());

    paths.put(
        "LowesHardware_outpostToDepotAround",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        // Outpost
                        PathWaypoint.from(LH_outpost).build(),
                        // Outside tower right
                        PathWaypoint.from(Tower.rightOutside.translation()).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // Outside tower left
                        PathWaypoint.from(Tower.leftOutside.translation()).build())
                    .pointAt(
                        new PointTarget(
                            FieldConstants.Hub.innerCenterPoint.toTranslation2d(),
                            Rotation2d.fromDegrees(1),
                            true))
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(
                        // After Tower Intermediate
                        PathWaypoint.from(new Translation2d(0.75, 5.0)).build())
                    .keepInLaneWidth(0.05)
                    .build())
            .segments(depotRightToLeft.build().segments.toArray(new PathRequestSegment[0]))
            .build());

    paths.put(
        "launchLeftTowerToClimbLeft",
        PathRequest.builder()
            .segments(
                PathRequestSegment.builder()
                    .waypoints(
                        PathWaypoint.from(Launch.leftTower).build(),
                        PathWaypoint.from(Climb.leftOffset).build())
                    .build(),
                PathRequestSegment.builder()
                    .waypoints(PathWaypoint.from(Climb.left).build())
                    .keepInLaneWidth(0.008)
                    .maxVelocity(0.5)
                    .maxAngularVelocity(0.01)
                    .build())
            .build());
  }
}
