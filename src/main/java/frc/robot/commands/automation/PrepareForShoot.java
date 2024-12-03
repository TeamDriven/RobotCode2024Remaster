package frc.robot.commands.automation;

import static frc.robot.subsystems.shooter.ShooterConstants.*;
import static frc.robot.Subsystems.*;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import java.util.function.DoubleSupplier;

/**
 * A command that prepares the robot for shooting by controlling the rotation, shooter speed, and
 * angle.
 */
public class PrepareForShoot extends ParallelCommandGroup {
  /**
   * Constructs a new PrepareForShoot command.
   *
   * @param rotation The rotation value for the AutoTurn command.
   * @param angle The angle value supplied by the DoubleSupplier.
   * @param speed The speed value supplied by the DoubleSupplier.
   */
  public PrepareForShoot(DoubleSupplier angle, DoubleSupplier speed, DoubleSupplier slapperAngle) {
    super(
        // new AutoTurnToGoal()
        new InstantCommand(() -> shooter.runShooter(speed.getAsDouble(), shooterSequenceAcceleration)),
        new InstantCommand(() -> angleController.setPosition(angle.getAsDouble())),
        new InstantCommand(() -> slapper.setPosition(slapperAngle.getAsDouble())));
  }
}

// public class PrepareForShoot extends ConditionalCommand {
//     /**
//      * Constructs a new PrepareForShoot command.
//      *
//      * @param rotation The rotation value for the AutoTurn command.
//      * @param angle The angle value supplied by the DoubleSupplier.
//      * @param speed The speed value supplied by the DoubleSupplier.
//      */
//     public PrepareForShoot(Double rotation, DoubleSupplier angle, DoubleSupplier speed){
//         super(
//             new ParallelCommandGroup(
//                 new ConditionalCommand(
//                     new AutoTurn(-rotation),
//                     new AutoTurn(rotation),
//                     () -> DriverStation.getAlliance().get().equals(DriverStation.Alliance.Red)
//                 ),
//                 shooter.speedUpShooterSupplier(speed, shooterSequenceAcceleration),
//                 angleController.setPositionCommandSupplier(angle)
//             ),
//             new ParallelCommandGroup(
//                 shooter.speedUpShooterSupplier(speed, shooterSequenceAcceleration),
//                 angleController.setPositionCommandSupplier(angle)
//             ),
//             () -> !rotation.isNaN()
//         );
//     }
// }
