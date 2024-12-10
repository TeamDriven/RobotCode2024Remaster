package frc.robot.subsystems.indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * The `Climber` class represents a subsystem that controls the climber mechanism of the robot. It
 * provides methods to control the climber motor, set the position of the climber, and run the
 * climber at a specified voltage.
 */
public class Indexer extends SubsystemBase {
  private final IndexerIO indexerIO;
  private final IndexerIOInputsAutoLogged indexerInputs = new IndexerIOInputsAutoLogged();

  private double velocity = 0;
  private double acceleration = 0;

  public Indexer(IndexerIO IndexerIO) {
    this.indexerIO = IndexerIO;
  }

  @Override
  public void periodic() {
    indexerIO.updateInputs(indexerInputs);
    Logger.processInputs("Indexer", indexerInputs);

    if (velocity == 0) {
      indexerIO.stopMotor();
    } else {
      indexerIO.runIndexer(velocity, acceleration);
    }
  }

  public void runIndexer(double velocity, double acceleration) {
    this.velocity = velocity;
    this.acceleration = acceleration;
  }

  public void stopMotor() {
    this.velocity = 0;
    this.acceleration = 0;
  }

  private boolean isAtSpeed(DoubleSupplier velocity) {
    if (velocity.equals(Double.NaN)) {
      return true;
    }
    return indexerIO.getVelocity() >= velocity.getAsDouble() * 0.90;
  }

  public Command checkIfAtSpeed(DoubleSupplier speed) {
    return new Command() {
      @Override
      public boolean isFinished() {
        return isAtSpeed(speed);
      }
    };
  }
}
