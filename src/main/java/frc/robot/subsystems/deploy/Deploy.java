package frc.robot.subsystems.deploy;

import edu.wpi.first.wpilibj.DriverStation;
// import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Deploy extends SubsystemBase {
  public DeployIO io;
  public DeployIOInputsAutoLogged inputs;

  private double setpoint;
  private DeployStates state;

  // private Timer homingTimer;

  public enum DeployStates {
    STARTING_CONFIG,
    REQUEST_SETPOINT
  }

  public Deploy(DeployIO deployIO) {
    this.io = deployIO;

    inputs = new DeployIOInputsAutoLogged();

    setpoint = 0;
    state = DeployStates.STARTING_CONFIG;

    // homingTimer = new Timer();
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Deploy", inputs);
    Logger.recordOutput("Deploy/Setpoint", setpoint);

    switch (state) {
      case STARTING_CONFIG:
        if (DriverStation.isEnabled()) {
          state = DeployStates.REQUEST_SETPOINT;
        }
        break;
      case REQUEST_SETPOINT:
        if (setpoint != 0.0) {
          io.setPosition(setpoint);
        }
        break;
    }
  }

  public void requestPosition(double position) {
    setpoint = position;
  }

  public double getPosition() {
    return inputs.pos;
  }

  public void seedPosition(double motorRotations) {
    io.seedPosition(motorRotations);
  }

  public void enableBrakeMode(boolean enable) {
    io.enableBrakeMode(enable);
  }

  public void stop() {
    io.stop();
  }
}
