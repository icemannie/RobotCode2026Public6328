// Copyright (c) 2025-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.littletonrobotics.frc2026;

import choreo.Choreo;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.io.File;
import java.lang.reflect.Method;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import lombok.Getter;
import lombok.Setter;
import lombok.experimental.ExtensionMethod;
import org.littletonrobotics.frc2026.Constants.Mode;
import org.littletonrobotics.frc2026.FieldConstants.AprilTagLayoutType;
import org.littletonrobotics.frc2026.commands.DriveCommands;
import org.littletonrobotics.frc2026.subsystems.drive.Drive;
import org.littletonrobotics.frc2026.subsystems.drive.DriveConstants;
import org.littletonrobotics.frc2026.subsystems.drive.GyroIO;
import org.littletonrobotics.frc2026.subsystems.drive.ModuleIO;
import org.littletonrobotics.frc2026.subsystems.drive.ModuleIOSim;
import org.littletonrobotics.frc2026.subsystems.hopper.Hopper;
import org.littletonrobotics.frc2026.subsystems.kicker.Kicker;
import org.littletonrobotics.frc2026.subsystems.launcher.LaunchCalculator;
import org.littletonrobotics.frc2026.subsystems.launcher.flywheel.Flywheel;
import org.littletonrobotics.frc2026.subsystems.launcher.flywheel.FlywheelIO;
import org.littletonrobotics.frc2026.subsystems.launcher.hood.Hood;
import org.littletonrobotics.frc2026.subsystems.launcher.hood.HoodIO;
import org.littletonrobotics.frc2026.subsystems.leds.Leds;
import org.littletonrobotics.frc2026.subsystems.leds.LedsIO;
import org.littletonrobotics.frc2026.subsystems.leds.LedsIOHAL;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystemIO;
import org.littletonrobotics.frc2026.subsystems.rollers.RollerSystemIOSim;
import org.littletonrobotics.frc2026.subsystems.sensors.FuelSensorIO;
import org.littletonrobotics.frc2026.subsystems.slamtake.SlamIO;
import org.littletonrobotics.frc2026.subsystems.slamtake.SlamIOSim;
import org.littletonrobotics.frc2026.subsystems.slamtake.Slamtake;
import org.littletonrobotics.frc2026.subsystems.slamtake.Slamtake.IntakeGoal;
import org.littletonrobotics.frc2026.subsystems.slamtake.Slamtake.SlamGoal;
import org.littletonrobotics.frc2026.subsystems.vision.Vision;
import org.littletonrobotics.frc2026.subsystems.vision.VisionIO;
import org.littletonrobotics.frc2026.util.FuelSim;
import org.littletonrobotics.frc2026.util.HubShiftUtil;
import org.littletonrobotics.frc2026.util.LoggedTunableNumber;
import org.littletonrobotics.frc2026.util.controllers.OverrideSwitches;
import org.littletonrobotics.frc2026.util.controllers.RazerWolverineController;
import org.littletonrobotics.frc2026.util.controllers.TriggerUtil;
import org.littletonrobotics.frc2026.util.geometry.AllianceFlipUtil;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

@ExtensionMethod({TriggerUtil.class})
public class RobotContainer {
  // Subsystems
  private Drive drive;
  private Slamtake slamtake;
  private Hopper hopper;
  private Kicker kicker;
  private Hood hood;
  private Flywheel flywheel;
  private Vision vision;
  private Leds leds;

  // Controller
  private final RazerWolverineController primary = new RazerWolverineController(0);
  private final CommandXboxController secondary = new CommandXboxController(1);
  private final OverrideSwitches overrides = new OverrideSwitches(5);

  private final Trigger superstructureCoast = overrides.driverSwitch(2);

  private final Alert primaryDisconnected =
      new Alert("Primary controller disconnected (port 0).", AlertType.kWarning);
  private final Alert secondaryDisconnected =
      new Alert("Secondary controller disconnected (port 1).", AlertType.kWarning);
  private final Alert overrideDisconnected =
      new Alert("Override controller disconnected (port 5).", AlertType.kInfo);
  private final Alert aprilTagLayoutAlert = new Alert("", AlertType.kInfo);

  // Dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser;
  private final LoggedDashboardChooser<AprilTagLayoutType> aprilTagLayoutChooser;
  private final LoggedTunableNumber presetHoodAngleDegrees =
      new LoggedTunableNumber("PresetHoodAngleDegrees", 30.0);
  private final LoggedTunableNumber presetFlywheelSpeedRadPerSec =
      new LoggedTunableNumber("PresetFlywheelSpeedRadPerSec", 290);

  private boolean coastOverride = false;

  /** Keeps track of the number of balls in the hopper with the fuel sim. */
  public class SimFuelCount {
    @Getter private static final int capacity = 60;
    @Getter private static final double launchBPS = 16.0;

    @Setter @Getter private int fuelStored;

    public SimFuelCount(int fuelStored) {
      this.fuelStored = fuelStored;
    }
  }

  private FuelSim fuelSim;
  private SimFuelCount simFuelCount;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure fuel sim
    if (Constants.getMode() == Mode.SIM) {
      fuelSim = new FuelSim("FuelSim");
      simFuelCount = new SimFuelCount(8);
      ObjectDetection.setFuelSim(fuelSim);
      configureFuelSim();
    }

    // Instantiate subsystems
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.robot) {
        case COMPBOT:
          // Not implemented
          break;

        case ALPHABOT:
          // Not implemented
          break;

        case SIMBOT:
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(0),
                  new ModuleIOSim(1),
                  new ModuleIOSim(2),
                  new ModuleIOSim(3));
          slamtake =
              new Slamtake(
                  new SlamIOSim(), new RollerSystemIOSim(DCMotor.getKrakenX60Foc(1), 1.0, 0.005));
          hopper =
              new Hopper(
                  new RollerSystemIO() {},
                  new FuelSensorIO() {},
                  new FuelSensorIO() {},
                  Optional.of(simFuelCount));
          kicker = new Kicker(new RollerSystemIO() {}, Optional.of(simFuelCount));
          leds = new Leds(new LedsIOHAL());
          break;
      }
    }

    // No-op implementations for replay
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }
    if (slamtake == null) {
      slamtake = new Slamtake(new SlamIO() {}, new RollerSystemIO() {});
    }
    if (hopper == null) {
      hopper =
          new Hopper(
              new RollerSystemIO() {},
              new FuelSensorIO() {},
              new FuelSensorIO() {},
              Optional.empty());
    }
    if (hood == null) {
      hood = new Hood(new HoodIO() {});
    }
    if (flywheel == null) {
      flywheel = new Flywheel("Flywheel", new FlywheelIO() {});
    }
    if (kicker == null) {
      kicker = new Kicker(new RollerSystemIO() {}, Optional.empty());
    }
    if (vision == null) {
      switch (Constants.robot) {
        case COMPBOT -> vision = new Vision(this::getSelectedAprilTagLayout);
        case ALPHABOT ->
            vision =
                new Vision(this::getSelectedAprilTagLayout, new VisionIO() {}, new VisionIO() {});
        default -> vision = new Vision(this::getSelectedAprilTagLayout);
      }
    }
    if (leds == null) {
      leds = new Leds(new LedsIO() {});
    }

    // Set up Choreo directory
    try {
      Method setChoreoDirMethod = Choreo.class.getDeclaredMethod("setChoreoDir", File.class);
      setChoreoDirMethod.setAccessible(true);
      setChoreoDirMethod.invoke(null, new File(Filesystem.getDeployDirectory(), "vts"));
    } catch (Exception e) {
      DriverStation.reportWarning("Failed to set Choreo directory.", false);
    }

    // Set up auto routines
    autoChooser = new LoggedDashboardChooser<>("Auto Choices");
    autoChooser.addDefaultOption("Do Nothing", Commands.none());
    autoChooser.addOption(
        "Drive Wheel Radius Characterization", DriveCommands.wheelRadiusCharacterization(drive));

    // Set up AprilTag layout type
    aprilTagLayoutChooser = new LoggedDashboardChooser<>("AprilTag Layout");
    aprilTagLayoutChooser.addDefaultOption("Official", FieldConstants.defaultAprilTagType);
    aprilTagLayoutChooser.addOption("Hub", AprilTagLayoutType.HUB);
    aprilTagLayoutChooser.addOption("Tower", AprilTagLayoutType.TOWER);
    aprilTagLayoutChooser.addOption("Outpost", AprilTagLayoutType.OUTPOST);
    aprilTagLayoutChooser.addOption("None", AprilTagLayoutType.NONE);

    hood.setCoastOverride(() -> coastOverride);

    // Configure the button bindings
    configureButtonBindings();

    // Set default commands
    hood.setDefaultCommand(hood.runTrackTargetCommand());
    flywheel.setDefaultCommand(flywheel.runTrackTargetCommand());
    slamtake.setDefaultCommand(
        Commands.startEnd(
            () -> {
              slamtake.setIntakeGoal(IntakeGoal.INTAKE);
              slamtake.setSlamGoal(SlamGoal.DEPLOY);
            },
            () -> {
              slamtake.setIntakeGoal(IntakeGoal.STOP);
              slamtake.setSlamGoal(SlamGoal.IDLE);
            },
            slamtake));
  }

  /** Create the bindings between buttons and commands. */
  private void configureButtonBindings() {
    // Drive controls
    DoubleSupplier driverX = () -> -primary.getLeftY() - secondary.getLeftY();
    DoubleSupplier driverY = () -> -primary.getLeftX() - secondary.getLeftX();
    DoubleSupplier driverOmega = () -> -primary.getRightX() - secondary.getRightX();
    drive.setDefaultCommand(DriveCommands.joystickDrive(drive, driverX, driverY, driverOmega));

    // ***** PRIMARY CONTROLLER *****
    primary
        .rightTrigger()
        .whileTrue(
            Commands.parallel(
                    Commands.startEnd(
                        () -> slamtake.setIntakeGoal(IntakeGoal.OUTTAKE),
                        () -> slamtake.setIntakeGoal(IntakeGoal.STOP),
                        slamtake),
                    Commands.startEnd(
                        () -> hopper.setGoal(Hopper.Goal.OUTTAKE),
                        () -> hopper.setGoal(Hopper.Goal.STOP),
                        hopper),
                    Commands.startEnd(
                        () -> kicker.setGoal(Kicker.Goal.OUTTAKE),
                        () -> kicker.setGoal(Kicker.Goal.STOP),
                        kicker))
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming));
    primary
        .leftClaw()
        .whileTrue(
            flywheel
                .runFixedCommand(presetFlywheelSpeedRadPerSec)
                .alongWith(
                    hood.runFixedCommand(
                        () -> Units.degreesToRadians(presetHoodAngleDegrees.get()), () -> 0.0)));
    primary
        .rightBumper()
        .whileTrue(DriveCommands.joystickDriveWhileLaunching(drive, driverX, driverY))
        .and(() -> LaunchCalculator.getInstance().getParameters().isValid())
        // .and(hood::atGoal)
        // .and(flywheel::atGoal)
        .and(() -> DriveCommands.atLaunchGoal())
        .debounce(0.1, DebounceType.kRising)
        .whileTrue(
            Commands.parallel(
                Commands.startEnd(
                    () -> hopper.setGoal(Hopper.Goal.LAUNCH),
                    () -> hopper.setGoal(Hopper.Goal.STOP),
                    hopper),
                Commands.startEnd(
                    () -> kicker.setGoal(Kicker.Goal.LAUNCH),
                    () -> kicker.setGoal(Kicker.Goal.STOP),
                    kicker)));
    // .onFalse(
    //     Commands.startEnd(
    //             () -> kicker.setGoal(Kicker.Goal.OUTTAKE),
    //             () -> kicker.setGoal(Kicker.Goal.STOP),
    //             kicker)
    //         .withTimeout(0.5));

    // Deploy intake
    primary.povUp().toggleOnTrue(Commands.runOnce(() -> slamtake.setSlamGoal(SlamGoal.DEPLOY)));

    // Retract intake
    primary.povDown().toggleOnTrue(Commands.runOnce(() -> slamtake.setSlamGoal(SlamGoal.RETRACT)));

    // ***** SECONDARY CONTROLLER *****

    // Reset gyro
    secondary
        .start()
        .and(secondary.back())
        .onTrue(
            Commands.runOnce(
                    () ->
                        RobotState.getInstance()
                            .resetPose(
                                new Pose2d(
                                    RobotState.getInstance().getEstimatedPose().getTranslation(),
                                    AllianceFlipUtil.apply(Rotation2d.kZero))))
                .withName("ResetGyro")
                .ignoringDisable(true));

    // Hood angle offset
    secondary
        .povUp()
        .whileTrue(
            Commands.runOnce(() -> LaunchCalculator.getInstance().incrementHoodAngleOffset(0.2))
                .andThen(
                    Commands.waitSeconds(0.3),
                    Commands.repeatingSequence(
                        Commands.runOnce(
                            () -> LaunchCalculator.getInstance().incrementHoodAngleOffset(0.2)),
                        Commands.waitSeconds(0.1)))
                .ignoringDisable(true));
    secondary
        .povDown()
        .whileTrue(
            Commands.runOnce(() -> LaunchCalculator.getInstance().incrementHoodAngleOffset(-0.2))
                .andThen(
                    Commands.waitSeconds(0.3),
                    Commands.repeatingSequence(
                        Commands.runOnce(
                            () -> LaunchCalculator.getInstance().incrementHoodAngleOffset(-0.2)),
                        Commands.waitSeconds(0.1)))
                .ignoringDisable(true));

    // ****** OVERRIDE SWITCHES *****

    // Coast superstructure
    superstructureCoast
        .onTrue(
            Commands.runOnce(
                    () -> {
                      if (DriverStation.isDisabled()) {
                        coastOverride = true;
                        leds.superstructureCoast = true;
                      }
                    })
                .withName("Superstructure Coast")
                .ignoringDisable(true))
        .onFalse(
            Commands.runOnce(
                    () -> {
                      coastOverride = false;
                      leds.superstructureCoast = false;
                    })
                .withName("Superstructure Uncoast")
                .ignoringDisable(true));
    RobotModeTriggers.disabled()
        .onFalse(
            Commands.runOnce(
                    () -> {
                      coastOverride = false;
                      leds.superstructureCoast = false;
                    })
                .ignoringDisable(true));
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(() -> HubShiftUtil.initialize()));
    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(() -> HubShiftUtil.initialize()));
  }

  private void configureFuelSim() {
    fuelSim.registerRobot(
        DriveConstants.fullWidthX,
        DriveConstants.fullWidthY,
        Units.inchesToMeters(6.0),
        () -> RobotState.getInstance().getEstimatedPose(),
        () -> RobotState.getInstance().getFieldVelocity());

    fuelSim.registerIntake(
        DriveConstants.intakeNearX,
        DriveConstants.intakeFarX,
        -DriveConstants.fullWidthY / 2,
        DriveConstants.fullWidthY / 2,
        () ->
            slamtake.getSlamGoal().equals(Slamtake.SlamGoal.DEPLOY)
                && slamtake.getIntakeGoal().equals(Slamtake.IntakeGoal.INTAKE)
                && simFuelCount.getFuelStored() < SimFuelCount.capacity,
        () ->
            simFuelCount.setFuelStored(
                Math.min(simFuelCount.getFuelStored() + 1, SimFuelCount.capacity)));

    fuelSim.setSubticks(1);
    fuelSim.start();
    fuelSim.spawnStartingFuel();

    RobotModeTriggers.autonomous()
        .onTrue(
            Commands.runOnce(
                () -> {
                  fuelSim.clearFuel();
                  fuelSim.spawnStartingFuel();
                  simFuelCount.setFuelStored(8);
                }));
  }

  public void updateFuelSim() {
    if (fuelSim != null) {
      fuelSim.updateSim();
    }
  }

  /** Update dashboard outputs. */
  public void updateDashboardOutputs() {
    // Publish match time
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());

    // Controller disconnected alerts
    primaryDisconnected.set(!DriverStation.isJoystickConnected(primary.getHID().getPort()));
    secondaryDisconnected.set(!DriverStation.isJoystickConnected(secondary.getHID().getPort()));
    overrideDisconnected.set(!overrides.isConnected());

    // AprilTag layout alert
    boolean aprilTagAlertActive = getSelectedAprilTagLayout() != FieldConstants.defaultAprilTagType;
    aprilTagLayoutAlert.set(aprilTagAlertActive);
    if (aprilTagAlertActive) {
      aprilTagLayoutAlert.setText(
          "Non-default AprilTag layout in use (" + getSelectedAprilTagLayout().toString() + ").");
    }
  }

  /** Returns the current AprilTag layout type. */
  public AprilTagLayoutType getSelectedAprilTagLayout() {
    return aprilTagLayoutChooser.get();
  }

  /** Returns the autonomous command for the Robot class. */
  public Command getAutonomousCommand() {
    return autoChooser.get();
  }
}
