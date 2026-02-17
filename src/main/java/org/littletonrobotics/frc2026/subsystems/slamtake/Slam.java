// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.slamtake;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.BooleanSupplier;
import lombok.Setter;
import org.littletonrobotics.frc2026.Robot;
import org.littletonrobotics.frc2026.subsystems.slamtake.SlamIO.SlamIOOutputMode;
import org.littletonrobotics.frc2026.subsystems.slamtake.SlamIO.SlamIOOutputs;
import org.littletonrobotics.junction.Logger;

public class Slam {
  private final SlamIO io;
  private final SlamIOInputsAutoLogged inputs = new SlamIOInputsAutoLogged();
  private final SlamIOOutputs outputs = new SlamIOOutputs();

  // Connected debouncer
  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert motorDisconnectedAlert =
      new Alert("Slam motor disconnected!", Alert.AlertType.kWarning);

  @Setter private BooleanSupplier coastOverride = () -> false;

  public Slam(SlamIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Slamtake/Slam", inputs);

    motorDisconnectedAlert.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connected));

    // Stop when disabled
    if (DriverStation.isDisabled()) {
      outputs.mode = SlamIOOutputMode.BRAKE;

      if (coastOverride.getAsBoolean()) {
        outputs.mode = SlamIOOutputMode.COAST;
      }
    }
  }

  public void periodicAfterScheduler() {
    io.applyOutputs(outputs);
  }

  public void runAmps(double amps) {
    outputs.mode = SlamIOOutputMode.RUN_OPEN_LOOP;
    outputs.appliedAmps = amps;
  }

  public void setMode(SlamIOOutputMode mode) {
    outputs.mode = mode;
  }

  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }
}
