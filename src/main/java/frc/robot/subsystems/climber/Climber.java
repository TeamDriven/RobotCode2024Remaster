package frc.robot.subsystems.climber;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.actuation.ActuationIO;
import frc.robot.subsystems.actuation.ActuationIOInputsAutoLogged;

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
    Logger.processInputs("Actuation", climberInputs);

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