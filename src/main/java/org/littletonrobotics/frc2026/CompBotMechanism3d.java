// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2026.subsystems.launcher.LauncherConstants;
import org.littletonrobotics.frc2026.util.geometry.GeomUtil;
import org.littletonrobotics.junction.Logger;

@ExtensionMethod({GeomUtil.class})
public class CompBotMechanism3d {
  private static CompBotMechanism3d measured;

  public static CompBotMechanism3d getMeasured() {
    if (measured == null) {
      measured = new CompBotMechanism3d();
    }
    return measured;
  }

  @Getter @Setter private Rotation2d hoodAngle = Rotation2d.kZero; // Relative to the ground
  @Getter @Setter private Rotation2d intakeAngle = Rotation2d.kZero; // Relative to the ground
  @Getter @Setter private double climberHeight = 0.5; // Ground to the hooks of the innter carriage

  /** Log the component poses and camera pose. */
  public void log(String key) {
    var hoodPose =
        LauncherConstants.robotToLauncher
            .toPose3d()
            .transformBy(
                new Transform3d(
                    0.0, 0.0, 0.0, new Rotation3d(0.0, -hoodAngle.getRadians(), Math.PI)));
    var intakePose =
        new Pose3d(0.266, 0.0, 0.2, new Rotation3d(0.0, -intakeAngle.getRadians(), 0.0));
    var climberPose =
        new Pose3d(-0.024, -0.273, climberHeight, new Rotation3d(0.0, 0.0, -Math.PI / 2.0));
    Logger.recordOutput(key + "/Components", hoodPose, intakePose, climberPose);
  }
}
