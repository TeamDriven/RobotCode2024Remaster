package frc.robot.subsystems.indexer;

import org.littletonrobotics.junction.AutoLog;

public interface IndexerIO {
  @AutoLog
  class IndexerIOInputs {
    public double speed = 0;
  }

  default void updateInputs(IndexerIOInputs inputs) {}

  default void runIndexer(double velocity, double acceleration) {}

  default double getVelocity() {return 0;}

  default void stopMotor() {}
}
