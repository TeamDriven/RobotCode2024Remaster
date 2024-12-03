// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot;

import static frc.robot.Constants.*;
import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.Controls.*;
import static frc.robot.Subsystems.*;
import static frc.robot.subsystems.angleController.AngleControllerConstants.*;
import static frc.robot.subsystems.indexer.IndexerConstants.*;
import static frc.robot.subsystems.intake.IntakeConstants.*;
import static frc.robot.subsystems.slapper.SlapperConstants.*;

import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import frc.robot.subsystems.shooter.ShooterConstants;
import frc.robot.commands.automation.AutoShootSequence;
import frc.robot.commands.automation.PickUpPiece;
import frc.robot.commands.automation.PickUpPieceAuto;
import frc.robot.commands.automation.PrepareForShoot;
import frc.robot.commands.automation.StopIntake;
import frc.robot.commands.automation.StopShoot;
import frc.robot.commands.automation.ZeroAngle;
// import frc.robot.commands.drivetrain.AutoTurnToGoal;
import frc.robot.commands.drivetrain.ResetDrive;
import frc.robot.subsystems.angleController.AngleControllerConstants;
import frc.robot.subsystems.slapper.SlapperConstants;
import frc.robot.util.*;
import frc.robot.util.Alert.AlertType;

public class RobotContainer {
  private final RobotState robotState = RobotState.getInstance();
  private static boolean isIntaking = false;
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

  private static SendableChooser<Command> autoChooser = new SendableChooser<>();

  // Dashboard inputs
  // private final AutoSelector autoSelector = new AutoSelector("Auto");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure autos and buttons
    linkAutoCommands();
    configureButtonBindings(false);

    autoChooser.setDefaultOption(
        "Do nothing",
        (Commands.runOnce(
                () ->
                    robotState.resetPose(
                        new Pose2d(
                            // robotState.getEstimatedPose().getTranslation(),
                            new Translation2d(), AllianceFlipUtil.apply(new Rotation2d()))))
            .ignoringDisable(true)));
    autoChooser.addOption("Blue 4 piece", drive.getAutoPath("close 4 blue"));
    autoChooser.addOption("Red 4 piece", drive.getAutoPath("close 4 red"));
    autoChooser.addOption("Blue Mobility", drive.getAutoPath("blue mobility"));
    autoChooser.addOption("Red Mobility", drive.getAutoPath("red mobility"));
    autoChooser.addOption("Red Shoot 1", drive.getAutoPath("shoot 1 red"));
    autoChooser.addOption("Blue Shoot 1", drive.getAutoPath("shoot 1 blue"));

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Alerts for constants
    if (Constants.tuningMode) {
      new Alert("Tuning mode enabled", AlertType.INFO).set(true);
    }
  }

  private Command driveCommand() {
    return drive
        .run(
            () ->
                drive.acceptTeleopInput(
                    driveX.getAsDouble(), driveY.getAsDouble(), driveOmega.getAsDouble(), false))
        .withName("Drive Teleop Input");
  }

  private void linkAutoCommands() {
    NamedCommands.registerCommand("zeroShooter", new ZeroAngle());

    NamedCommands.registerCommand(
        "stopDrive", new InstantCommand(() -> drive.clearAutoInput(), drive));

    NamedCommands.registerCommand(
        "shoot1CloseBlue",
        new AutoShootSequence(
            () -> -1.25, () -> 45, 17.5, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand(
        "shoot2CloseBlue",
        new AutoShootSequence(
            () -> 17.5, () -> 45, 18.5, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand(
        "shoot3CloseBlue",
        new AutoShootSequence(
            () -> 18.5, () -> 45, 19.5, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand(
        "shoot4CloseBlue",
        new AutoShootSequence(
            () -> 19.5,
            () -> 45,
            AngleControllerConstants.restingPosition,
            () -> slapperRestingPosition,
            slapperRestingPosition));

    NamedCommands.registerCommand(
        "shoot1CloseRed",
        new AutoShootSequence(
            () -> -1.0, () -> 40, 17.5, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand(
        "shoot2CloseRed",
        new AutoShootSequence(
            () -> 17.5, () -> 45, 18.5, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand(
        "shoot3CloseRed",
        new AutoShootSequence(
            () -> 18.5, () -> 45, 19.5, () -> slapperRestingPosition, slapperRestingPosition));
    NamedCommands.registerCommand(
        "shoot4CloseRed",
        new AutoShootSequence(
            () -> 19.5,
            () -> 45,
            AngleControllerConstants.restingPosition,
            () -> slapperRestingPosition,
            slapperRestingPosition));

    NamedCommands.registerCommand("intake", new PickUpPieceAuto(autoIntakeVoltage));
    NamedCommands.registerCommand("stopIntake", new StopIntake());
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

    resetPose.onTrue(
        Commands.runOnce(
                () ->
                    robotState.resetPose(
                        new Pose2d(
                            // robotState.getEstimatedPose().getTranslation(),
                            new Translation2d(), AllianceFlipUtil.apply(new Rotation2d()))))
            .ignoringDisable(true));

    // runIntake.onTrue(new PickUpPiece(intakeVoltage)).onFalse(new StopIntake());

    runIntake.onTrue(new InstantCommand(() -> isIntaking = !isIntaking));

    new Trigger(() -> isIntaking)
        .onTrue(
            new PickUpPiece(intakeVoltage).andThen(new InstantCommand(() -> isIntaking = false)));

    new Trigger(() -> !isIntaking).onTrue(new StopIntake());

    climberUp.whileTrue(new InstantCommand(() -> climber.runVoltage(12)));

    climberDown.whileTrue(new InstantCommand(() -> climber.runVoltage(-12)));

    // driver.y().whileTrue(slapper.runVoltageCommand(-0.1));
    // driver.a().whileTrue(slapper.runVoltageCommand(0.1));

    manualOut
        .whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> intake.runVoltage(-4)),
                new InstantCommand(
                    () -> indexer.runIndexer(-indexerVelocity, indexerAcceleration))))
        .onFalse(new InstantCommand(() -> intake.stopMotor()));

    manualIn
        .whileTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> intake.runVoltage(4)),
                new InstantCommand(() -> indexer.runIndexer(indexerVelocity, indexerAcceleration))))
        .onFalse(new InstantCommand(() -> intake.stopMotor()));

    // On Stop Shooting
    new Trigger(() -> currentShootingState.equals(shootingState.IDLE))
        .onTrue(
            new StopShoot(
                AngleControllerConstants.restingPosition, SlapperConstants.slapperRestingPosition));

    subwooferShot.onTrue(
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
                    AngleControllerConstants.restingPosition,
                    () -> slapperRestingPosition,
                    slapperRestingPosition)
                .andThen(new InstantCommand(this::stopShooting)));

    podiumShot.onTrue(
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
            new SequentialCommandGroup(
                // new AutoTurnToGoal(() -> podiumShotOffset), sorry they no want auto turn
                new AutoShootSequence(
                    () -> podiumShotAngle,
                    () -> podiumShotAngle,
                    AngleControllerConstants.restingPosition,
                    () -> slapperRestingPosition,
                    slapperRestingPosition),
                new InstantCommand(this::stopShooting),
                new ResetDrive()));
    passShot.onTrue(
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
            new SequentialCommandGroup(
                // new AutoTurnToGoal(() -> passShotOffset),
                new AutoShootSequence(
                    () -> passShotAngle,
                    () -> passShotSpeed,
                    AngleControllerConstants.restingPosition,
                    () -> slapperRestingPosition,
                    slapperRestingPosition),
                new InstantCommand(this::stopShooting),
                new ResetDrive()));

    ampShot.onTrue(
        new ConditionalCommand(
            new InstantCommand(this::incrementShootingMode),
            setShootingTypeCommand(shootingType.AMP),
            () -> currentShootingType.equals(shootingType.AMP)));

    new Trigger(() -> currentShootingType.equals(shootingType.AMP))
        .and(() -> currentShootingState.equals(shootingState.PREPARED))
        .onTrue(
            new ParallelCommandGroup(
                new InstantCommand(() -> drive.setHeadingGoal(() -> Rotation2d.fromDegrees(90))),
                new InstantCommand(() -> angleController.setPosition(ampAngle)),
                new InstantCommand(() -> slapper.setPosition(slapperAmpPosition))))
        .onFalse(new InstantCommand(() -> drive.clearHeadingGoal()));

    new Trigger(() -> currentShootingType.equals(shootingType.AMP))
        .and(() -> currentShootingState.equals(shootingState.SHOOTING))
        .onTrue(
            new ParallelCommandGroup(
                new AutoShootSequence(
                        () -> ampAngle,
                        () -> ampSpeed,
                        AngleControllerConstants.restingPosition,
                        () -> slapperAmpPosition,
                        slapperPushNotePosition)
                    .andThen(
                        new SequentialCommandGroup(
                                new InstantCommand(
                                    () -> slapper.setPosition(slapperPostAmpPosition)),
                                new WaitCommand(0.75),
                                new InstantCommand(this::stopShooting))
                            .beforeStarting(
                                new InstantCommand(() -> slapper.waitUntilAtPosition())))));

    cancelShot.onTrue(new InstantCommand(this::stopShooting));
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
    // Characterization
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
    // return drive.getAutoPath("close 4 blue");
    return autoChooser.getSelected();
  }
}
