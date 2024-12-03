package frc.robot.subsystems.climber;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * The `Climber` class represents a subsystem that controls the climber mechanism of the robot. It
 * provides methods to control the climber motor, set the position of the climber, and run the
 * climber at a specified voltage.
 */
public class Climber extends SubsystemBase {
  private final ClimberIO climberIO;
  private final ClimberIOInputsAutoLogged climberInputs = new ClimberIOInputsAutoLogged();

  private double voltage = 0;

  public Climber(ClimberIO climberIO) {
    this.climberIO = climberIO;
  }

  @Override
  public void periodic() {
    climberIO.updateInputs(climberInputs);
    Logger.processInputs("Climber", climberInputs);

    if (voltage == 0) {
      climberIO.stopMotor();
    } else {
      climberIO.runVoltage(voltage);
    }
  }

  public void runVoltage(double voltage) {
    this.voltage = voltage;
  }
}
