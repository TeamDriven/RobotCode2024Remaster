package frc.robot.subsystems.limelightIntake;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.limelightIntake.LimelightIntakeIOLimelight.IntakeLightMode;

public interface LimelightIntakeIO {
  @AutoLog
  class LimelightIntakeIOInputs {
    public double TX;
    public double TY;
    public double TA;
    public double TS;
  }

  default void updateInputs(LimelightIntakeIOInputs inputs) {}

  default void turnOnLimelight() {}

  default void turnOffLimelight() {}

  default void setLights(IntakeLightMode lightMode) {}

  default double[] getRobotPose() { return new double[6];}

  default double getApriltagID() { return 0; }
}
