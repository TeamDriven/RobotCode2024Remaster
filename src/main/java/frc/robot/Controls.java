package frc.robot;

import static frc.robot.Constants.driver;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import java.util.function.DoubleSupplier;

public class Controls {
  private static final boolean rightStickDrive = true;

  // Drivetrain
  public static DoubleSupplier driveX =
      () -> rightStickDrive ? -driver.getRightX() : -driver.getLeftX();
  public static DoubleSupplier driveY =
      () -> rightStickDrive ? -driver.getRightY() : -driver.getLeftY();
  public static DoubleSupplier driveOmega =
      () -> rightStickDrive ? -driver.getLeftX() : -driver.getRightX();
  public static Trigger resetPose = driver.start();

  // Intake
  public static Trigger runIntake = driver.rightBumper();
  public static Trigger manualIn = driver.x();
  public static Trigger manualOut = driver.b();

  // Climber
  public static Trigger climberUp = driver.pov(0);
  public static Trigger climberDown = driver.pov(180);

  // Shooting
  public static Trigger subwooferShot = driver.rightTrigger(0.1);
  public static Trigger podiumShot = driver.leftTrigger(0.1);
  public static Trigger passShot = driver.leftBumper();
  public static Trigger ampShot = driver.pov(270);

  public static Trigger cancelShot = driver.pov(90);
}
