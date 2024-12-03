package frc.robot.subsystems.limelightIntake;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.slapper.SlapperIO.SlapperIOInputs;

public class LimelightIntakeIOLimelight implements LimelightIntakeIO {
  public final String LIMELIGHT = "limelight-intake";
  NetworkTable table = NetworkTableInstance.getDefault().getTable(LIMELIGHT);

  public static enum LightMode {
    DEFAULT(0),
    OFF(1),
    BLINK(2),
    ON(3);

    public int lightNum;

    private LightMode(int lightNum) {
      this.lightNum = lightNum;
    }
  };

  public static enum Pipeline {
    Note(0);
    // AprilTag3D(1);

    public int pipelineNum;

    private Pipeline(int pipelineNum) {
      this.pipelineNum = pipelineNum;
    }
  }

  public LimelightIntakeIOLimelight() {
    turnOnLimelight();
  }

  public void updateInputs(LimelightIntakeIOInputs inputs) {}

  public void turnOnLimelight() {
    NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("camMode").setNumber(0);
    setLights(LightMode.DEFAULT);
  }

  /** Switches the camera to inactive and turns off the lights */
  public void turnOffLimelight() {
    NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("camMode").setNumber(1);
    setLights(LightMode.OFF);
  }

  public void setLights(LightMode lightMode) {
    NetworkTableInstance.getDefault()
        .getTable(LIMELIGHT)
        .getEntry("ledMode")
        .setNumber(lightMode.lightNum);
  }

  public double getApriltagID() {
    double id = NetworkTableInstance.getDefault().getTable(LIMELIGHT).getEntry("tid").getDouble(0);
    // SmartDashboard.putNumber("Apriltag id", id);
    return id;
    // return 1;
  }

  public double[] getRobotPose() {
    double[] pose = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    try {
      pose =
          NetworkTableInstance.getDefault()
              .getTable(LIMELIGHT)
              .getEntry("botpose")
              .getDoubleArray(pose);
    } catch (ArrayIndexOutOfBoundsException e) {}
    return pose;
    // return null;
  }

   public void setLimelightPipeline(Pipeline pipeline) {
    NetworkTableInstance.getDefault()
        .getTable(LIMELIGHT)
        .getEntry("pipeline")
        .setNumber(pipeline.pipelineNum);
  }
  
  public void prepareForNote() {
    turnOnLimelight();
    setLimelightPipeline(Pipeline.Note);
  }
}
