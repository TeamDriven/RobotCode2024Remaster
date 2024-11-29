package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  class IntakeIOInputs {
    public double motorVelocity = 0;
  }

  default void updateInputs(IntakeIOInputs inputs) {}
  
  default void feedMotor(double velocity, double acceleration) {}

  default void stopMotor() {}

  default boolean getNoteSensor() {return false;}

  default void runVoltage(double voltage) {}
}
