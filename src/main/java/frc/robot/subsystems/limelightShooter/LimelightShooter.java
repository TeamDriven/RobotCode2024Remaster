package frc.robot.subsystems.limelightShooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.limelightShooter.LimelightShooterIOLimelight.ShooterLightMode;

/**
 * The `Slapper` class represents a subsystem that controls the Slapper mechanism of the robot. It
 * provides methods to control the Slapper motor, set the position of the Slapper, and run the
 * Slapper at a specified voltage.
 */
public class LimelightShooter extends SubsystemBase {
  private final LimelightShooterIO limelightShooterIO;
  private final LimelightShooterIOInputsAutoLogged limelightIntakeInputs = new LimelightShooterIOInputsAutoLogged();

  private ShooterLightMode mode = ShooterLightMode.DEFAULT;

  private boolean isOn = false;

  public LimelightShooter(LimelightShooterIO limelightShooterIO) {
    this.limelightShooterIO = limelightShooterIO;
  }

  @Override
  public void periodic() {
    limelightShooterIO.updateInputs(limelightIntakeInputs);
    Logger.processInputs("LimelightIntake", limelightIntakeInputs);

    limelightShooterIO.setLights(mode);

    if (this.isOn) {
      limelightShooterIO.turnOnLimelight();
    } else {
      limelightShooterIO.turnOffLimelight();
    }
  }

  public void setLights(ShooterLightMode inputMode) {
    this.mode = inputMode;
  }

  public void turnOnLimelight() {
    this.isOn = true;
  }

  public void turnOffLimelight() {
    this.isOn = false;
  }

}
