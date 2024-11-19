package frc.robot.subsystems.AngleController;

import org.littletonrobotics.junction.AutoLog;

public interface AngleControllerIO {
  @AutoLog
  class AngleControllerIOInputs {
    public double motorPosition = 0;
    public boolean sensorIsTripped = false;
  }

  default void updateInputs(AngleControllerIOInputs inputs) {}

  default void setPosition(double pos) {}

  default double getPosition() { return 0; }

  default void stopMotor() {}
}
