package frc.robot.subsystems.angleController;

import org.littletonrobotics.junction.AutoLog;

public interface AngleControllerIO {
  @AutoLog
  class AngleControllerIOInputs {
    public double motorPosition = 0;
    public boolean sensorIsTripped = false;
  }

  default void updateInputs(AngleControllerIOInputs inputs) {}

  default void setPosition(double pos) {}

  default double getPosition() {
    return 0;
  }

  default boolean getSensor() {
    return false;
  }

  default void setOnSensor() {}

  default void stopMotor() {}
}
