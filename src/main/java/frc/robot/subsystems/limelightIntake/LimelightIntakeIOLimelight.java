package frc.robot.subsystems.limelightIntake;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

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

  public void updateInputs(LimelightIntakeIOInputs inputs) {
    inputs.TA = getTA();
    inputs.TS = getTS();
    inputs.TX = getTX();
    inputs.TY = getTY();
  }

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
