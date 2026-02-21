// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026.subsystems.launcher.flywheel;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.experimental.Accessors;
import org.littletonrobotics.frc2026.Robot;
import org.littletonrobotics.frc2026.subsystems.launcher.LaunchCalculator;
import org.littletonrobotics.frc2026.subsystems.launcher.flywheel.FlywheelIO.FlywheelIOOutputMode;
import org.littletonrobotics.frc2026.subsystems.launcher.flywheel.FlywheelIO.FlywheelIOOutputs;
import org.littletonrobotics.frc2026.util.EqualsUtil;
import org.littletonrobotics.frc2026.util.FullSubsystem;
import org.littletonrobotics.frc2026.util.LoggedTracer;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Flywheel extends FullSubsystem {
  private final FlywheelIO io;
  private final FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();
  private final FlywheelIOOutputs outputs = new FlywheelIOOutputs();

  private final Debouncer motorConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer motorFollower1ConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer motorFollower2ConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer motorFollower3ConnectedDebouncer =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Alert disconnected;
  private final Alert follower1Disconnected;
  private final Alert follower2Disconnected;
  private final Alert follower3Disconnected;

  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Flywheel/kP", 0.4);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Flywheel/kD", 0.0);
  private static final LoggedTunableNumber kS = new LoggedTunableNumber("Flywheel/kS", 0.4);
  private static final LoggedTunableNumber kV = new LoggedTunableNumber("Flywheel/kV", 0.02);
  private static final LoggedTunableNumber maxAcceleration =
      new LoggedTunableNumber("Flywheel/MaxAcceleration", 50.0);

  private SlewRateLimiter slewRateLimiter = new SlewRateLimiter(maxAcceleration.get());

  @Getter
  @Accessors(fluent = true)
  @AutoLogOutput(key = "Flywheel/AtGoal")
  private boolean atGoal = false;

  public Flywheel(FlywheelIO io) {
    this.io = io;

    disconnected = new Alert("Flywheel motor disconnected!", Alert.AlertType.kWarning);
    follower1Disconnected =
        new Alert("Flywheel follower 1 motor disconnected!", Alert.AlertType.kWarning);
    follower2Disconnected =
        new Alert("Flywheel follower 2 motor disconnected!", Alert.AlertType.kWarning);
    follower3Disconnected =
        new Alert("Flywheel follower 3 motor disconnected!", Alert.AlertType.kWarning);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Flywheel", inputs);

    outputs.kP = kP.get();
    outputs.kD = kD.get();

    if (maxAcceleration.hasChanged(hashCode())) {
      slewRateLimiter = new SlewRateLimiter(maxAcceleration.get());
    }

    if (DriverStation.isDisabled()) {
      stop();
    }

    disconnected.set(
        Robot.showHardwareAlerts() && !motorConnectedDebouncer.calculate(inputs.connected));
    follower1Disconnected.set(
        Robot.showHardwareAlerts()
            && !motorFollower1ConnectedDebouncer.calculate(inputs.follower1Connected));
    follower2Disconnected.set(
        Robot.showHardwareAlerts()
            && !motorFollower2ConnectedDebouncer.calculate(inputs.follower2Connected));
    follower3Disconnected.set(
        Robot.showHardwareAlerts()
            && !motorFollower3ConnectedDebouncer.calculate(inputs.follower3Connected));

    LoggedTracer.record("Flywheel/Periodic");
  }

  @Override
  public void periodicAfterScheduler() {
    Logger.recordOutput("Flywheel/Mode", outputs.mode);
    io.applyOutputs(outputs);

    LoggedTracer.record("Flywheel/AfterScheduler");
  }

  /** Run closed loop at the specified velocity. */
  private void runVelocity(double velocityRadsPerSec) {
    double setpointRadPerSec = slewRateLimiter.calculate(velocityRadsPerSec);
    atGoal = EqualsUtil.epsilonEquals(setpointRadPerSec, velocityRadsPerSec, 2.0);
    outputs.mode = FlywheelIOOutputMode.VELOCITY;
    outputs.velocityRadsPerSec = setpointRadPerSec;
    outputs.feedforward = Math.signum(setpointRadPerSec) * kS.get() + setpointRadPerSec * kV.get();
    Logger.recordOutput("Flywheel/Setpoint", setpointRadPerSec);
    Logger.recordOutput("Flywheel/Goal", velocityRadsPerSec);
  }

  /** Stops the flywheel. */
  private void stop() {
    outputs.mode = FlywheelIOOutputMode.COAST;
    outputs.velocityRadsPerSec = 0.0;
    atGoal = false;
    slewRateLimiter.reset(inputs.velocityRadsPerSec);
  }

  /** Returns the current velocity in RPM. */
  public double getVelocity() {
    return inputs.velocityRadsPerSec;
  }

  public Command runTrackTargetCommand() {
    return runEnd(
        () -> runVelocity(LaunchCalculator.getInstance().getParameters().flywheelSpeed()),
        this::stop);
  }

  public Command runFixedCommand(DoubleSupplier velocity) {
    return runEnd(() -> runVelocity(velocity.getAsDouble()), this::stop);
  }

  public Command stopCommand() {
    return runOnce(this::stop);
  }
}
