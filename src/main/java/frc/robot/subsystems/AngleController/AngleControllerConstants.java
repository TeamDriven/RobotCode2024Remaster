package frc.robot.subsystems.angleController;

public class AngleControllerConstants {
  public static final double rotationsPerDegree =
      (11.78 * 1.75 / 360 * 4) / 1.5; // Estimate given by Adam, ask him how to do it if you need

  public static final double startingPosition = 0;
  public static final double restingPosition = 15;

  public static final double trapAngle = 0;
  public static double ampAngle = 0;

  public static double subwooferShotAngle = -1.25;
  public static double podiumShotAngle = 23.0; // 23 Half Line Shot 34.75
  public static double chainShotAngle = 33.25;
  public static double championshipShotAngle = 33;
  public static double passShotAngle = 15;
}
