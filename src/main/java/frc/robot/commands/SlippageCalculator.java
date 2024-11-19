// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import static frc.robot.subsystems.drive.DriveConstants.kinematics;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.Arrays;
import java.util.LinkedList;
import java.util.List;

public class SlippageCalculator extends Command {
  private static final double linearSpeed = 1;
  private static final double angularSpeed = 1;

  private SlipData data;

  private final Drive drivetrain;

  private Timer timer = new Timer();

  /** Creates a new FeedForwardCharacterization command. */
  public SlippageCalculator(Drive drivetrain) {
    this.drivetrain = drivetrain;
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    data = new SlipData();
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drivetrain.acceptSimpleInput(linearSpeed, 0, angularSpeed, false);
    ChassisSpeeds desiredSpeed = drivetrain.getSimpleSpeeds();
    SwerveModuleState[] moduleState = drivetrain.getModuleStates();
    data.add(moduleState, desiredSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.acceptSimpleInput(0, 0, 0, false);
    timer.stop();
    data.print();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 1.5;
  }

  public static class SlipData {
    private final List<SwerveModuleState[]> moduleStates = new LinkedList<>();
    private final List<ChassisSpeeds> desiredSpeeds = new LinkedList<>();

    public void add(SwerveModuleState[] moduleState, ChassisSpeeds desiredSpeed) {
      moduleStates.add(moduleState);
      desiredSpeeds.add(desiredSpeed);
    }

    public void print() {
      if (moduleStates.size() == 0 || desiredSpeeds.size() == 0) {
        return;
      }

      SwerveModuleState[][] stateData = moduleStates.toArray(new SwerveModuleState[0][]);
      ChassisSpeeds[] speedData = desiredSpeeds.toArray(new ChassisSpeeds[0]);

      double differenceAverage = 0;
      for (int i = 0; i < stateData.length; i++) {
        double[] moduleSpeeds = new double[4];
        SwerveModuleState[] turnState =
            kinematics.toSwerveModuleStates(
                new ChassisSpeeds(0, 0, speedData[i].omegaRadiansPerSecond));
        for (int j = 0; j < stateData[i].length; j++) {
          Translation2d currentVector =
              new Translation2d(stateData[i][j].speedMetersPerSecond, stateData[i][j].angle);
          Translation2d turnVector =
              new Translation2d(turnState[j].speedMetersPerSecond, turnState[j].angle);
          Translation2d linearVector = currentVector.minus(turnVector);
          moduleSpeeds[j] = linearVector.getNorm();
        }
        double max = Arrays.stream(moduleSpeeds).max().getAsDouble();
        double average = Arrays.stream(moduleSpeeds).sum();
        average -= max;
        average /= 3;
        differenceAverage += max - average;
      }
      differenceAverage /= stateData.length;

      System.out.println("Slippage Calculator Results:");
      System.out.println("\tCount=" + stateData.length);
      System.out.println("\tAverage Difference=" + differenceAverage);
    }
  }
}
