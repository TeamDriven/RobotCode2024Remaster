package frc.robot.subsystems.actuation;

public class ActuationConstants {
  private static final double motorGearRatio = 8;
  private static final double encoderGearRatio = 2;

  public static final double motorRotationsPerDegree = 1.0 / 360.0 * motorGearRatio;
  public static final double encoderRotationsPerDegree = 1.0 / 360.0 * encoderGearRatio;

  public static final double startPosition = -65;
  public static final double pickUpPosition = 98; // 100
  public static final double tuckPosition = -63.3;

  public static final double offset = 1.9 - startPosition;
}
