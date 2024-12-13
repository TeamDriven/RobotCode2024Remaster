// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.limelightIntake.LimelightIntake;
import frc.robot.subsystems.limelightIntake.LimelightIntakeIO;
import frc.robot.subsystems.limelightIntake.LimelightIntakeIOLimelight;
import frc.robot.subsystems.limelightShooter.LimelightShooter;
import frc.robot.subsystems.limelightShooter.LimelightShooterIO;
import frc.robot.subsystems.limelightShooter.LimelightShooterIOLimelight;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.actuation.Actuation;
import frc.robot.subsystems.actuation.ActuationIO;
import frc.robot.subsystems.actuation.ActuationIOFalcon500;
import frc.robot.subsystems.angleController.AngleController;
import frc.robot.subsystems.angleController.AngleControllerIO;
import frc.robot.subsystems.angleController.AngleControllerIOKraken;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.climber.ClimberIOKraken;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOKrakenFOC;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.indexer.IndexerIOKraken;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOKraken;
import frc.robot.subsystems.shooter.ShooterIOKraken;
import frc.robot.subsystems.slapper.Slapper;
import frc.robot.subsystems.slapper.SlapperIO;
import frc.robot.subsystems.slapper.SlapperIOKraken;

/**
 * The Subsystems class represents the collection of subsystems used in the robot. It provides
 * static references to various subsystem objects that are used in the robot.
 */
public final class Subsystems {
  public static final Intake intake; // My intake
  public static final Shooter shooter; // My shooter
  public static final Indexer indexer; // My indexer
  public static final Climber climber; // My climber
  public static final AngleController angleController; // My angle controller
  public static final LimelightShooter limelightShooter; // My limelight for the shooter
  public static final LimelightIntake limelightIntake; // My limelight for the intake
  public static final Slapper slapper; // My slapper

  public static final Drive drive;
  public static final Actuation actuation; // My actuation

  static {
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
          actuation = new Actuation(new ActuationIOFalcon500(14, 6));
          shooter = new Shooter(new ShooterIOKraken(15, 16));
          climber = new Climber(new ClimberIOKraken(18));
          indexer = new Indexer(new IndexerIOKraken(17));
          angleController = new AngleController(new AngleControllerIOKraken(19, 14));
          slapper = new Slapper(new SlapperIOKraken(20, 5));
          intake = new Intake(new IntakeIOKraken(13, 0, 1));
          limelightIntake = new LimelightIntake(new LimelightIntakeIOLimelight());
          limelightShooter = new LimelightShooter(new LimelightShooterIOLimelight());
        }
        case DEVBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(false),
                  new ModuleIOSparkMax(DriveConstants.moduleConfigs[0]),
                  new ModuleIOSparkMax(DriveConstants.moduleConfigs[1]),
                  new ModuleIOSparkMax(DriveConstants.moduleConfigs[2]),
                  new ModuleIOSparkMax(DriveConstants.moduleConfigs[3]));
          actuation = new Actuation(new ActuationIOFalcon500(14, 6));
          shooter = new Shooter(new ShooterIOKraken(15, 16));
          climber = new Climber(new ClimberIOKraken(18));
          indexer = new Indexer(new IndexerIOKraken(17));
          angleController = new AngleController(new AngleControllerIOKraken(19, 14));
          slapper = new Slapper(new SlapperIOKraken(20, 5));
          intake = new Intake(new IntakeIOKraken(13, 0, 1));
          limelightIntake = new LimelightIntake(new LimelightIntakeIOLimelight());
          limelightShooter = new LimelightShooter(new LimelightShooterIOLimelight());
        }
        case SIMBOT -> {
          throw new IllegalStateException("SIMBOT is not currently implemented on this robot");
        }
        default -> {
          throw new IllegalStateException("Robot type not selected");
        }
      }
    } else {
      drive =
          new Drive(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
      actuation = new Actuation(new ActuationIO() {});
      climber = new Climber(new ClimberIO() {});
      angleController = new AngleController(new AngleControllerIO() {});
      slapper = new Slapper(new SlapperIO() {});
      indexer = new Indexer(new IndexerIO() {});
      intake = new Intake(new IntakeIO() {});
      shooter = new Shooter(new ShooterIO() {});
      limelightIntake = new LimelightIntake(new LimelightIntakeIO() {});
      limelightShooter = new LimelightShooter(new LimelightShooterIO() {});
    }
  }
}
