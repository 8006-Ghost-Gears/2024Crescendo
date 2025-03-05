package frc.robot.subsystems.deploy;

import org.littletonrobotics.junction.AutoLog;

public interface DeployIO {
  @AutoLog
  public static class DeployIOInputs {
    public double pos = 0.0;
    public double appliedVoltage = 0.0;
    public double supplyCurrentAmps = 0.0; // {leader, follower}
    public double[] statorCurrentAmps = new double[] {}; // {leader, follower}
    public double[] tempCelsius = new double[] {}; // {leader, follower}
  }

  public default void updateInputs(DeployIOInputs inputs) {}

  public default void setPosition(double position) {}

  public default void setVoltage(double voltage) {}

  public default void seedPosition(double motorPositionRot) {}

  public default void stop() {}

  public default void enableBrakeMode(boolean enable) {}
}
