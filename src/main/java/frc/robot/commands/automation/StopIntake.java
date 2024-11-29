package frc.robot.commands.automation;

import static frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.actuation.Actuation.position;

/** A command group that stops the intake and sets the actuation position to a tuck position. */
public class StopIntake extends ParallelCommandGroup {
  public StopIntake() {
    addCommands(
      new InstantCommand(() -> intake.stopMotor()), new InstantCommand(() -> actuation.setPosition(position.TUCK)));
  }
}
