package frc.robot.subsystems.limelightShooter;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimelightShooterIOLimelight implements LimelightShooterIO {
  public final static String LIMELIGHT = "limelight-shooter";
  NetworkTable table = NetworkTableInstance.getDefault().getTable(LIMELIGHT);

  public static enum ShooterLightMode {
    DEFAULT(0),
    OFF(1),
    BLINK(2),
    ON(3);

    public int lightNum;

    private ShooterLightMode(int lightNum) {
      this.lightNum = lightNum;
    }
  };

  public static enum ShooterPipeline {
    PoseEstimation(0);

    public int pipelineNum;

    private ShooterPipeline(int pipelineNum) {
      this.pipelineNum = pipelineNum;
    }
  }


  public LimelightShooterIOLimelight() {
    turnOnLimelight();
  }

  public void updateInputs(LimelightShooterIOInputs inputs) {
    inputs.TA = getTA();
    inputs.TS = getTS();
    inputs.TX = getTX();
    inputs.TY = getTY();
  }

  public void turnOnLimelight() {
    table.getEntry("camMode").setNumber(0);
    setLights(ShooterLightMode.DEFAULT);
  }

  /** Switches the camera to inactive and turns off the lights */
  public void turnOffLimelight() {
    table.getEntry("camMode").setNumber(1);
    setLights(ShooterLightMode.OFF);
  }

  public void setLights(ShooterLightMode lightMode) {
    table.getEntry("ledMode").setNumber(lightMode.lightNum);
  }


  public Double getApriltagID() {
    double id = table.getEntry("tid").getDouble(0);
    return (id != 0) ? id : Double.NaN;
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
    } catch (ArrayIndexOutOfBoundsException e) {
      // System.out.println("No 3D April tag detected");
    }
    return pose;
    // return null;
  }

  public void setLimelightPipeline(ShooterPipeline pipeline) {
    table.getEntry("pipeline").setNumber(pipeline.pipelineNum);
  }

  
  public Double getTX() {
    double tX = table.getEntry("tx").getDouble(0);
    return (tX != 0) ? tX : Double.NaN;
    // return 1;
  }

  /**
   * @return Y position of the object (degrees)
   */
  public Double getTY() {
    double tY = table.getEntry("ty").getDouble(0);
    return (tY != 0) ? tY : Double.NaN;
    // return 1;
  }

  /**
   * @return Area of the screen the object takes up
   */
  public Double getTA() {
    double tA = table.getEntry("ta").getDouble(0);
    return (tA != 0) ? tA : Double.NaN;
    // return 1;
  }

  /**
   * @return Skew (rotation) of the object
   */
  public Double getTS() {
    double tS = table.getEntry("ts").getDouble(0);
    return (tS != 0) ? tS : Double.NaN;
    // return 1;
  }
}
