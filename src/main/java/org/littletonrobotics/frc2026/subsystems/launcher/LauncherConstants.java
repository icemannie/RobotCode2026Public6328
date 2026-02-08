// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.launcher;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class LauncherConstants {
  // TODO: update this from double launcher CAD
  public static Transform3d robotToLauncher =
      new Transform3d(-0.19685, 0.0, 0.44, Rotation3d.kZero);

  private LauncherConstants() {}
}
