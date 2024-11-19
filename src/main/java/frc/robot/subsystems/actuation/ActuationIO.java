package frc.robot.subsystems.actuation;

import org.littletonrobotics.junction.AutoLog;

public interface ActuationIO {
  @AutoLog
  class ActuationIOInputs {
    public double motorAngle = 0;
    public double encoderAngle = 0;
  }
  
  default void updateInputs(ActuationIOInputs inputs) {}

  default void setAngle(double angle) {}

  default void runVoltage(double voltage) {}

  default void stopMotor() {}

  default void syncPosition(double encoderAngle) {}
}
