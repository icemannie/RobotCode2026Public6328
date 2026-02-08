// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026;

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
public class AlphaMechanism3d {
  private static AlphaMechanism3d measured;

  public static AlphaMechanism3d getMeasured() {
    if (measured == null) {
      measured = new AlphaMechanism3d();
    }
    return measured;
  }

  @Getter @Setter private Rotation2d hoodAngle = Rotation2d.kZero; // Relative to the ground

  /** Log the component poses and camera pose. */
  public void log(String key) {
    var hoodPose =
        LauncherConstants.robotToLauncher
            .toPose3d()
            .transformBy(
                new Transform3d(
                    0.0, 0.0, 0.0, new Rotation3d(0.0, -hoodAngle.getRadians(), Math.PI)));
    Logger.recordOutput(key + "/Components", hoodPose);
  }
}
