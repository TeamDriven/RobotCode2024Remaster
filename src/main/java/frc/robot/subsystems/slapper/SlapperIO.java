package frc.robot.subsystems.slapper;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.controller.PIDController;

public interface SlapperIO {
  @AutoLog
  class SlapperIOInputs {
    public double motorPosition = 0;
    public double motorVelocity = 0;
    public double encoderAngle = 0;
  }

  default void updateInputs(SlapperIOInputs inputs) {}

  default void runVoltage(double voltage) {}

  default double getPosition() {return 0;}

  default void resetPosition() {}

  default void setPosition(double position) {}

  default void runMotorToPosition(double desiredPos, double feedForward, PIDController posPID) {}

  default void stopMotor() {}
}
