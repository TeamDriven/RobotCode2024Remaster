package frc.robot.subsystems.slapper;

import static frc.robot.subsystems.climber.ClimberConstants.*;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.actuation.ActuationIO;
import frc.robot.subsystems.actuation.ActuationIOInputsAutoLogged;
import frc.robot.subsystems.slapper.SlapperIO.SlapperIOInputs;

/**
 * The `Climber` class represents a subsystem that controls the climber mechanism of the robot. It
 * provides methods to control the climber motor, set the position of the climber, and run the
 * climber at a specified voltage.
 */
public class Slapper extends SubsystemBase {
  private final SlapperIO slapperIO;
  private final SlapperIOInputsAutoLogged climberInputs = new SlapperIOInputsAutoLogged();

  private double voltage = 0;

  private boolean isPositionControl = false;
  private double desiredPos = 0;

  private PIDController posPID = new PIDController(0.05, 0, 0.0025);
  private double feedForward = -0.3;

  public Slapper(SlapperIO slapperIO) {
    this.slapperIO = slapperIO;
  }
  
  @Override
  public void periodic() {
    slapperIO.updateInputs(SlapperIOInputs);
    Logger.processInputs("Actuation", climberInputs);

    if (isPositionControl) {
      slapperIO.runMotorToPosition();
    }else if (voltage == 0) {
      slapperIO.stopMotor();
    } else {
      slapperIO.runVoltage(voltage);
    }
  }

  public void runVoltage(double voltage) {
    this.voltage = voltage;
    //isPositionControl = false;
  }

  public void setPosition(double position) {
    desiredPos = position;
    isPositionControl = true;
  }
}
