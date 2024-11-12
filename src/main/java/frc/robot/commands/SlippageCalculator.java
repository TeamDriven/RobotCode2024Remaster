// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.drive.Drive;
import java.util.LinkedList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class SlippageCalculator extends Command {
  private static final double voltage = 12;

  private SlipData data;

  private final Drive drivetrain;

  private ChassisSpeeds prevChassisSpeeds = new ChassisSpeeds();

  private static Timer timer = new Timer();

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
    drivetrain.runCharacterization(voltage);
    ChassisSpeeds curChassisSpeeds = drivetrain.getSpeeds();
    Translation2d pigeyAcceleration = drivetrain.getAcceleration().toTranslation2d();

    double calculatedAcceleration =
        curChassisSpeeds.vxMetersPerSecond - prevChassisSpeeds.vxMetersPerSecond;
    calculatedAcceleration /= timer.get();
    data.add(calculatedAcceleration, pigeyAcceleration.getX());

    Logger.recordOutput("Slippage/calculatedAcceleration", calculatedAcceleration);
    Logger.recordOutput("Slippage/pigeyAcceleration", pigeyAcceleration);

    timer.reset();
    prevChassisSpeeds = curChassisSpeeds;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    timer.stop();
    data.print();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public static class SlipData {
    private final List<Double> calculatedAccelerationData = new LinkedList<>();
    private final List<Double> pigeyAccelerationData = new LinkedList<>();

    public void add(double calculatedAcceleration, double pigeyAcceleration) {
      if (Math.abs(calculatedAcceleration) > 1E-4) {
        calculatedAccelerationData.add(Math.abs(calculatedAcceleration));
        pigeyAcceleration =
            Units.MetersPerSecond.convertFrom(Math.abs(pigeyAcceleration), Units.Gs.getUnit());
        pigeyAccelerationData.add(pigeyAcceleration);
      }
    }

    public void print() {
      if (calculatedAccelerationData.size() == 0 || pigeyAccelerationData.size() == 0) {
        return;
      }

      double[] calcAccelData =
          calculatedAccelerationData.stream().mapToDouble(Double::doubleValue).toArray();
      double[] pigeyAccelData =
          pigeyAccelerationData.stream().mapToDouble(Double::doubleValue).toArray();

      double differenceAverage = 0;
      for (int i = 0; i < calcAccelData.length; i++) {
        differenceAverage += Math.abs(calcAccelData[i] - pigeyAccelData[i]);
      }
      differenceAverage /= calcAccelData.length;

      System.out.println("Slippage Calculator Results:");
      System.out.println("\tCount=" + calcAccelData.length);
      System.out.println("\tAverage Difference=" + differenceAverage);
    }
  }
}
