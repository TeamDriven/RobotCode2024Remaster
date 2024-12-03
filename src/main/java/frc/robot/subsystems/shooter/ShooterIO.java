package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  class ShooterIOInputs {
    public double lSpeed = 0;
    public double rSpeed = 0;
  }

  default void updateInputs(ShooterIOInputs inputs) {}

  default void runShooter(double velocity, double acceleration) {}

  default void runShooterSlow(double velocity, double acceleration) {}

  default boolean getNoteSensor() { return false;}

  default void sitMode() {}

  default void stopMotor() {}
}
