package frc.robot.subsystems.shooter;

import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

/**
 * The `Climber` class represents a subsystem that controls the climber mechanism of the robot. It
 * provides methods to control the climber motor, set the position of the climber, and run the
 * climber at a specified voltage.
 */
public class Shooter extends SubsystemBase {
  private final ShooterIO shooterIO;
  private final ShooterIOInputsAutoLogged shooterInputs = new ShooterIOInputsAutoLogged();

  private double voltage = 0;
  private double acceleration = 0;

  enum ShooterState {
    STOP,
    SLOW,
    FAST,
    SIT;
  }

  private ShooterState mode;

  public Shooter(ShooterIO climberIO) {
    this.shooterIO = climberIO;
  }

  @Override
  public void periodic() {
    shooterIO.updateInputs(shooterInputs);
    Logger.processInputs("Shooter", shooterInputs);
    
    Logger.recordOutput("Shooter/LeftVelocity", shooterInputs.lSpeed);
    Logger.recordOutput("Shooter/RightVelocity", shooterInputs.rSpeed);

    if (mode == ShooterState.STOP) {
      shooterIO.stopMotor();
    } else if(mode == ShooterState.FAST) {
      shooterIO.runShooter(voltage, acceleration);
    }else if(mode == ShooterState.SLOW) {
      shooterIO.runShooterSlow(voltage, acceleration);
    }else if(mode == ShooterState.SIT) {
      shooterIO.sitMode();
    }
  }

  public void runShooter(double voltage, double acceleration) {
    this.voltage = voltage;
    this.acceleration = acceleration;
    mode = ShooterState.FAST;
  }

  public void runShooterSlow(double voltage, double acceleration) {
    this.voltage = voltage;
    this.acceleration = acceleration;
    mode = ShooterState.SLOW;
  }

  public void sitMode() {
    mode = ShooterState.SIT;
  }

  public void stopMotors() {
    mode = ShooterState.STOP;
  }

  /**
   * Wait until the note sensor is tripped
   *
   * @return a command that will wait until the note sensor is tripped
   */
  public Command waitUntilTripped() {
    return new Command() {
      @Override
      public boolean isFinished() {
        return shooterIO.getNoteSensor();
      }
    };
  }

  /**
   * Wait until the note sensor is not tripped
   *
   * @return a command that will wait until the note sensor is not tripped
   */
  public Command waitUntilNotTripped() {
    return new Command() {
      @Override
      public boolean isFinished() {
        return !shooterIO.getNoteSensor();
      }
    };
  }

  public SequentialCommandGroup waitUntilRingLeft() {
    return new SequentialCommandGroup(waitUntilTripped(), waitUntilNotTripped());
  }

  public Command checkIfAtSpeed(double velocity) {
    return new Command() {
      @Override
      public void initialize() {}

      @Override
      public void execute() {}

      @Override
      public void end(boolean interrupted) {}

      @Override
      public boolean isFinished() {
        boolean leftShooterSpeed = shooterInputs.lSpeed >= velocity;
        boolean rightShooterSpeed = shooterInputs.rSpeed >= velocity;
        return leftShooterSpeed && rightShooterSpeed;
      }
    };
  }
}
