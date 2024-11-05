package frc.robot.subsystems.climber;

import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
  @AutoLog
  class ClimberIOInputs {
    public double position = 0;
  }

  default void updateInputs(ClimberIOInputs inputs) {}

  default void runVoltage(double voltage) {}

  default void runUnsafeVoltage(double voltage) {}

  default void stopMotor() {}
}
