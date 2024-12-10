package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.actuation.Actuation.position;

/**
 * This class represents a command group for picking up a piece in auto. It extends the
 * SequentialCommandGroup class.
 */
public class PickUpPieceAuto extends SequentialCommandGroup {

  /**
   * Constructs a new PickUpPieceAuto object with the specified voltage.
   *
   * @param voltage the voltage to be used for running the intake
   */
  public PickUpPieceAuto(double voltage) {
    addCommands(
        new InstantCommand(() -> actuation.setPosition(position.PICK_UP)),
        actuation.waitUntilAtPosition(),
        new InstantCommand(() -> intake.runVoltage(voltage)),
        intake.waitUntilTripped(),
        new InstantCommand(() -> intake.stopMotor()),
        new InstantCommand(() -> actuation.setPosition(position.TUCK)));
  }
}
