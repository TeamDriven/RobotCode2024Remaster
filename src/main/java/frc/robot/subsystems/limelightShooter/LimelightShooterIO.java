package frc.robot.subsystems.limelightShooter;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.limelightShooter.LimelightShooterIOLimelight.ShooterLightMode;

public interface LimelightShooterIO {
  @AutoLog
  class LimelightShooterIOInputs {
    public double TX;
    public double TY;
    public double TA;
    public double TS;
  }

  default void updateInputs(LimelightShooterIOInputs inputs) {}

  default void turnOnLimelight() {}

  default void turnOffLimelight() {}

  default void setLights(ShooterLightMode lightMode) {}

  default double[] getRobotPose() { return new double[6];}

  default Double getApriltagID() { return 0.0; }
}
