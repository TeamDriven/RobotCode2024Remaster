package frc.robot.subsystems.AngleController;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.climber.ClimberIO;

import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/** The AngleController class represents a subsystem that controls the angle of the shooter. */
public class AngleController extends SubsystemBase {
  private final AngleControllerIO angleControllerIO;
  private final AngleControllerIOInputsAutoLogged angleControllerInputs = new AngleControllerIOInputsAutoLogged();

  private double position = 0;

  public AngleController(AngleControllerIO angleControllerIO) {
    this.angleControllerIO = angleControllerIO;
  }
  
  @Override
  public void periodic() {
    angleControllerIO.updateInputs(angleControllerInputs);
    Logger.processInputs("AngleController", angleControllerInputs);

    if (position != angleControllerIO.getPosition()) {
      angleControllerIO.stopMotor();
    } else {
      angleControllerIO.setPosition(position);
    }
  }

  public void setPosition(double pos) {
    this.position = pos;
  }
}
