// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.actuation.Actuation;
import frc.robot.subsystems.actuation.ActuationIO;
import frc.robot.subsystems.angleController.AngleController;
import frc.robot.subsystems.angleController.AngleControllerIO;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.climber.ClimberIO;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOKrakenFOC;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.indexer.IndexerIO;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.slapper.Slapper;
import frc.robot.subsystems.slapper.SlapperIO;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIO;
import frc.robot.subsystems.vision.VisionIOLimelight;

/**
 * The Subsystems class represents the collection of subsystems used in the robot. It provides
 * static references to various subsystem objects that are used in the robot.
 */
public final class Subsystems {
  public static final Actuation actuation = new Actuation(new ActuationIO() {}); // My actuation
  public static final Intake intake = new Intake(new IntakeIO() {}); // My intake
  public static final Shooter shooter = new Shooter(new ShooterIO() {}); // My shooter
  public static final Indexer indexer = new Indexer(new IndexerIO() {}); // My indexer
  public static final Climber climber = new Climber(new ClimberIO() {}); // My climber
  public static final AngleController angleController = new AngleController(new AngleControllerIO() {}); // My angle controller
  public static final Slapper slapper = new Slapper(new SlapperIO() {}); // My slapper

  public static final Drive drive;
  public static final Vision mainVision;

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
          mainVision = new Vision("MainVision", new VisionIOLimelight("limelight"), drive::getSpeeds);
          // actuation = new Actuation(new ActuationIOFalcon500(14, 6));
          // shooter = new Shooter(new ShooterIOKraken(15, 16));
          // climber = new Climber(new ClimberIOKraken(18));
          // indexer = new Indexer(new IndexerIOKraken(17));
          // angleController = new AngleController(new AngleControllerIOKraken(19, 14));
          // slapper = new Slapper(new SlapperIOKraken(20, 5));
          // intake = new Intake(new IntakeIOKraken(13, 0, 1));
        }
        case DEVBOT -> {
          drive =
              new Drive(
                  new GyroIOPigeon2(false),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[0]),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[1]),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[2]),
                  new ModuleIOKrakenFOC(DriveConstants.moduleConfigs[3]));
          mainVision = new Vision("MainVision", new VisionIOLimelight("limelight"), drive::getSpeeds);
          // actuation = new Actuation(new ActuationIOFalcon500(14, 6));
          // shooter = new Shooter(new ShooterIOKraken(15, 16));
          // climber = new Climber(new ClimberIOKraken(18));
          // indexer = new Indexer(new IndexerIOKraken(17));
          // angleController = new AngleController(new AngleControllerIOKraken(19, 14));
          // slapper = new Slapper(new SlapperIOKraken(20, 5));
          // intake = new Intake(new IntakeIOKraken(13, 0, 1));
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
      mainVision = new Vision("MainVision", new VisionIO() {}, drive::getSpeeds);
      // actuation = new Actuation(new ActuationIO() {});
      // climber = new Climber(new ClimberIO() {});
      // angleController = new AngleController(new AngleControllerIO() {});
      // slapper = new Slapper(new SlapperIO() {});
      // indexer = new Indexer(new IndexerIO() {});
      // intake = new Intake(new IntakeIO() {});
      // shooter = new Shooter(new ShooterIO() {});
    }
  }
}
