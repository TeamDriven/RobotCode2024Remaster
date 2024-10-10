// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.Constants.ActuationConstants.*;
import static frc.robot.Constants.AngleControllerConstants.*;
import static frc.robot.Constants.IndexerConstants.*;
import static frc.robot.Constants.IntakeConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.SlapperConstants.*;
import static frc.robot.Subsystems.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.AngleControllerConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.SlapperConstants;
import frc.robot.commands.automation.AutoShootSequence;
import frc.robot.commands.automation.PickUpPiece;
import frc.robot.commands.automation.PrepareForShoot;
import frc.robot.commands.automation.StopIntake;
import frc.robot.commands.automation.StopShoot;
import frc.robot.subsystems.drive.*;
import frc.robot.util.*;
import frc.robot.util.Alert.AlertType;

public class RobotContainer {
  private final RobotState robotState = RobotState.getInstance();
  private static boolean isIntaking = false;
  public static final boolean rightStickDrive = false;
  private final Alert driverDisconnected =
      new Alert("Driver controller disconnected (port 0).", AlertType.WARNING);

  private static enum shootingState {
    IDLE,
    PREPARED,
    SHOOTING;
  };

  private static shootingState currentShootingState = shootingState.IDLE;

  private static enum shootingType {
    PODIUM,
    SUBWOOFER,
    PASS,
    AMP;
  };

  private static shootingType currentShootingType = shootingType.SUBWOOFER;

  // Dashboard inputs
  // private final AutoSelector autoSelector = new AutoSelector("Auto");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Create subsystems
    if (Constants.getMode() != Constants.Mode.REPLAY) {
      switch (Constants.getRobot()) {
        case COMPBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(true),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[0]),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[1]),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[2]),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[3]));
        }
        case DEVBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(false),
                  new ModuleIOSparkMax(DriveConstants.moduleConfigs[0]),
                  new ModuleIOSparkMax(DriveConstants.moduleConfigs[1]),
                  new ModuleIOSparkMax(DriveConstants.moduleConfigs[2]),
                  new ModuleIOSparkMax(DriveConstants.moduleConfigs[3]));
        }
        case SIMBOT -> {
          drive =
              new Drive(
                  new GyroIO() {},
                  new ModuleIOSim(DriveConstants.moduleConfigs[0]),
                  new ModuleIOSim(DriveConstants.moduleConfigs[1]),
                  new ModuleIOSim(DriveConstants.moduleConfigs[2]),
                  new ModuleIOSim(DriveConstants.moduleConfigs[3]));
        }
      }
    }

    // No-op implementation for replay
    if (drive == null) {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    }

    // Configure autos and buttons
    configureButtonBindings(false);

    // Alerts for constants
    if (Constants.tuningMode) {
      new Alert("Tuning mode enabled", AlertType.INFO).set(true);
    }
  }

  private Command driveCommand() {
    if (rightStickDrive) {
      return drive
          .run(
              () ->
                  drive.acceptTeleopInput(
                      -driver.getRightY(), -driver.getRightX(), -driver.getLeftX(), false))
          .withName("Drive Teleop Input");
    } else {
      return drive
          .run(
              () ->
                  drive.acceptTeleopInput(
                      -driver.getLeftY(), -driver.getLeftX(), -driver.getRightX(), false))
          .withName("Drive Teleop Input");
    }
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link Joystick} or {@link
   * XboxController}), and then passing it to a {@link JoystickButton}.
   */
  private void configureButtonBindings(boolean demo) {
    CommandScheduler.getInstance().getActiveButtonLoop().clear();

    // ------------- Driver Controls -------------
    drive.setDefaultCommand(driveCommand());

    driver
        .start()
        .onTrue(
            Commands.runOnce(
                    () ->
                        robotState.resetPose(
                            new Pose2d(
                                // robotState.getEstimatedPose().getTranslation(),
                                new Translation2d(), AllianceFlipUtil.apply(new Rotation2d()))))
                .ignoringDisable(true));

    // driver
    //     .rightBumper()
    //     .whileTrue(new PickUpPiece(IntakeConstants.intakeVoltage))
    //     .onFalse(new StopIntake());

    // driver
    //     .rightBumper()
    //     .onTrue(
    //         new ConditionalCommand(
    //             new ParallelCommandGroup(
    //                 new StopIntake(),
    //                 new InstantCommand(
    //                     () -> {
    //                       isIntaking = false;
    //                       System.out.println(isIntaking);
    //                     })),
    //             new ParallelCommandGroup(
    //                 new PickUpPiece(intakeVoltage),
    //                 new InstantCommand(
    //                     () -> {
    //                       isIntaking = true;
    //                       System.out.println(isIntaking);
    //                     })),
    //             () -> isIntaking));

    driver.rightBumper().onTrue(new InstantCommand(() -> isIntaking = !isIntaking));

    new Trigger(() -> isIntaking)
        .onTrue(
            new PickUpPiece(intakeVoltage).andThen(new InstantCommand(() -> isIntaking = false)));

    new Trigger(() -> !isIntaking).onTrue(new StopIntake());

    driver.pov(0).whileTrue(climber.runLimitedVoltageCommand(12));

    driver.pov(180).whileTrue(climber.runLimitedVoltageCommand(-12));

    driver
        .b()
        .whileTrue(
            new ParallelCommandGroup(
                intake.runVoltageCommand(-4),
                indexer.runIndexerCommand(-indexerVelocity, indexerAcceleration)))
        .onFalse(intake.stopIntakeCommand());

    driver
        .x()
        .whileTrue(
            new ParallelCommandGroup(
                intake.runVoltageCommand(4),
                indexer.runIndexerCommand(indexerVelocity, indexerAcceleration)))
        .onFalse(intake.stopIntakeCommand());

    // On Stop Shooting
    new Trigger(() -> currentShootingState.equals(shootingState.IDLE))
        .onTrue(
            new StopShoot(
                AngleControllerConstants.angleRestingPosition,
                SlapperConstants.slapperRestingPosition));

    driver
        .rightTrigger(0.1)
        .onTrue(
            new ConditionalCommand(
                new InstantCommand(this::incrementShootingMode),
                setShootingTypeCommand(shootingType.SUBWOOFER),
                () -> currentShootingType.equals(shootingType.SUBWOOFER)));

    new Trigger(() -> currentShootingState.equals(shootingState.PREPARED))
        .and(() -> currentShootingType.equals(shootingType.SUBWOOFER))
        .onTrue(
            new PrepareForShoot(
                () -> subwooferShotAngle, () -> subwooferShotSpeed, () -> slapperRestingPosition));

    new Trigger(() -> currentShootingState.equals(shootingState.SHOOTING))
        .and(() -> currentShootingType.equals(shootingType.SUBWOOFER))
        .onTrue(
            new AutoShootSequence(
                    () -> subwooferShotAngle,
                    () -> subwooferShotSpeed,
                    angleRestingPosition,
                    () -> slapperRestingPosition,
                    slapperRestingPosition)
                .andThen(new InstantCommand(this::stopShooting)));

    driver
        .leftTrigger(0.1)
        .onTrue(
            new ConditionalCommand(
                new InstantCommand(this::incrementShootingMode),
                setShootingTypeCommand(shootingType.PODIUM),
                () -> currentShootingType.equals(shootingType.PODIUM)));

    new Trigger(() -> currentShootingState.equals(shootingState.PREPARED))
        .and(() -> currentShootingType.equals(shootingType.PODIUM))
        .onTrue(
            new PrepareForShoot(
                () -> AngleControllerConstants.podiumShotAngle,
                () -> ShooterConstants.podiumShotSpeed,
                () -> SlapperConstants.slapperRestingPosition));

    new Trigger(() -> currentShootingState.equals(shootingState.SHOOTING))
        .and(() -> currentShootingType.equals(shootingType.PODIUM))
        .onTrue(
            new AutoShootSequence(
                    () -> podiumShotAngle,
                    () -> podiumShotAngle,
                    angleRestingPosition,
                    () -> slapperRestingPosition,
                    slapperRestingPosition)
                .andThen(new InstantCommand(this::stopShooting)));
    driver
        .leftBumper()
        .onTrue(
            new ConditionalCommand(
                new InstantCommand(this::incrementShootingMode),
                setShootingTypeCommand(shootingType.PASS),
                () -> currentShootingType.equals(shootingType.PASS)));

    new Trigger(() -> currentShootingState.equals(shootingState.PREPARED))
        .and(() -> currentShootingType.equals(shootingType.PASS))
        .onTrue(
            new PrepareForShoot(
                () -> AngleControllerConstants.passShotAngle,
                () -> ShooterConstants.passShotSpeed,
                () -> SlapperConstants.slapperRestingPosition));

    new Trigger(() -> currentShootingState.equals(shootingState.SHOOTING))
        .and(() -> currentShootingType.equals(shootingType.PASS))
        .onTrue(
            new AutoShootSequence(
                    () -> passShotAngle,
                    () -> passShotSpeed,
                    angleRestingPosition,
                    () -> slapperRestingPosition,
                    slapperRestingPosition)
                .andThen(new InstantCommand(this::stopShooting)));

    driver
        .pov(270)
        .onTrue(
            new ConditionalCommand(
                new InstantCommand(this::incrementShootingMode),
                setShootingTypeCommand(shootingType.AMP),
                () -> currentShootingType.equals(shootingType.AMP)));

    // new Trigger(() -> currentShootingState.equals(shootingState.PREPARED))
    //     .and(() -> currentShootingType.equals(shootingType.AMP))
    //     .onTrue(
    //         new PrepareForShoot(
    //             () -> AngleControllerConstants.ampAngle,
    //             () -> ShooterConstants.ampSpeed,
    //             () -> SlapperConstants.slapperAmpPosition));

    // new Trigger(() -> currentShootingState.equals(shootingState.SHOOTING))
    //     .and(() -> currentShootingType.equals(shootingType.AMP))
    //     .onTrue(
    //         new AutoShootSequence(
    //                 () -> ampAngle,
    //                 () -> ampSpeed,
    //                 angleRestingPosition,
    //                 () -> slapperAmpPosition,
    //                 slapperRestingPosition)
    //             .andThen(new InstantCommand(this::stopShooting)));

    new Trigger(() -> currentShootingType.equals(shootingType.AMP))
        .and(() -> currentShootingState.equals(shootingState.PREPARED))
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> drive.setHeadingGoal(() -> Rotation2d.fromDegrees(90))),
                angleController.setPositionCommandSupplier(() -> ampAngle),
                slapper.setPositionCommand(slapperAmpPosition)))
        .onFalse(new InstantCommand(() -> drive.clearHeadingGoal()));

    new Trigger(() -> currentShootingType.equals(shootingType.AMP))
        .and(() -> currentShootingState.equals(shootingState.SHOOTING))
        .onTrue(
            new ParallelCommandGroup(
                new AutoShootSequence(
                        () -> ampAngle,
                        () -> ampSpeed,
                        angleRestingPosition,
                        () -> slapperAmpPosition,
                        slapperPushNotePosition)
                    .andThen(
                        new SequentialCommandGroup(
                                slapper.setPositionCommand(slapperPostAmpPosition),
                                new WaitCommand(0.75),
                                new InstantCommand(this::stopShooting))
                            .beforeStarting(
                                slapper.waitUntilAtPosition(slapperPushNotePosition)))));

    driver.pov(90).onTrue(new InstantCommand(this::stopShooting));
  }

  /**
   * Increments the shooting mode to the next state. The shooting mode follows the sequence: IDLE ->
   * PREPARED -> SHOOTING -> IDLE.
   */
  public void incrementShootingMode() {
    currentShootingState =
        switch (currentShootingState) {
          case IDLE -> shootingState.PREPARED;
          case PREPARED -> shootingState.SHOOTING;
          case SHOOTING -> shootingState.IDLE;
          default -> shootingState.IDLE;
        };
  }

  /**
   * Sets the shooting type for the robot.
   *
   * @param type The shooting type to set.
   */
  public void setShootingType(shootingType type) {
    currentShootingType = type;
    currentShootingState = shootingState.PREPARED;
  }

  /**
   * Sets the shooting type command.
   *
   * @param type the shooting type to set
   * @return the command object that sets the shooting type
   */
  public Command setShootingTypeCommand(shootingType type) {
    return new Command() {
      @Override
      public void execute() {
        setShootingType(type);
      }

      @Override
      public boolean isFinished() {
        return true;
      }
    };
  }

  /** sets the shooting state to the IDLE. */
  public void stopShooting() {
    currentShootingState = shootingState.IDLE;
  }

  /** Updates the alerts for disconnected controllers. */
  public void checkControllers() {
    driverDisconnected.set(
        !DriverStation.isJoystickConnected(driver.getHID().getPort())
            || !DriverStation.getJoystickIsXbox(driver.getHID().getPort()));
  }

  /** Updates dashboard data. */
  public void updateDashboardOutputs() {
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {

    // Drive Static
    // Characterizationy0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000
    // return new StaticCharacterization(
    //         drive, drive::runCharacterization, drive::getCharacterizationVelocity)
    //     .finallyDo(drive::endCharacterization);

    // Drive FF Characterization
    // return new FeedForwardCharacterization(
    //         drive, drive::runCharacterization, drive::getCharacterizationVelocity)
    //     .finallyDo(drive::endCharacterization);

    // Drive Wheel Radius Characterization
    // return drive
    //     .orientModules(Drive.getCircleOrientations())
    //     .andThen(
    //         new WheelRadiusCharacterization(
    //             drive, WheelRadiusCharacterization.Direction.COUNTER_CLOCKWISE))
    //     .withName("Drive Wheel Radius Characterization");
    return drive.getAutoPath("New Auto");
  }
}
