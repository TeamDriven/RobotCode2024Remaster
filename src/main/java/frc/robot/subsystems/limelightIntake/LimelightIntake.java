package frc.robot.subsystems.limelightIntake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.limelightIntake.LimelightIntakeIOLimelight.IntakeLightMode;

/**
 * The `Slapper` class represents a subsystem that controls the Slapper mechanism of the robot. It
 * provides methods to control the Slapper motor, set the position of the Slapper, and run the
 * Slapper at a specified voltage.
 */
public class LimelightIntake extends SubsystemBase {
  private final LimelightIntakeIO limelightIntakeIO;
  private final LimelightIntakeIOInputsAutoLogged limelightIntakeInputs = new LimelightIntakeIOInputsAutoLogged();

  private IntakeLightMode mode = IntakeLightMode.DEFAULT;

  public LimelightIntake(LimelightIntakeIO limelightIntakeIO) {
    this.limelightIntakeIO = limelightIntakeIO;
  }

  @Override
  public void periodic() {
    limelightIntakeIO.updateInputs(limelightIntakeInputs);
    Logger.processInputs("LimelightIntake", limelightIntakeInputs);

    limelightIntakeIO.setLights(mode);
  }

  public void setLights(IntakeLightMode inputMode) {
    this.mode = inputMode;
  }

}
