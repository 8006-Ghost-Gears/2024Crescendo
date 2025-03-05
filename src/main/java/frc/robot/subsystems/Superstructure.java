package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.deploy.Deploy;
import frc.robot.subsystems.elevator.Elevator;

import org.littletonrobotics.junction.Logger;

public class Superstructure extends SubsystemBase {
  private boolean requestIdle;
  private boolean requestFeed;
  private boolean requestFEEDINGS;
  private boolean requestElevate;
  private boolean requestElevateIdle;
  

  private Superstates state;
  private Deploy deploy;
  private Elevator elevator;

  public static enum Superstates {
    IDLE,
    FEEDING,
    FEEDINGS,
    ELEVATE,
    ELEVATE_IDLE
  }

  public Superstructure(Deploy deploy, Elevator elevator) {
    this.deploy = deploy;
    this.elevator = elevator;
    state = Superstates.IDLE;
  }

  @Override
  public void periodic() {
    Logger.recordOutput("Superstructure/State", state.toString());
    switch (state) {
      case IDLE:
        deploy.requestPosition(0.0001);
        elevator.requestPosition(0.0001);
        if (requestFeed) {
            state = Superstates.FEEDING;
        }
        if (requestFEEDINGS) {
          state = Superstates.FEEDINGS;
        }
        break;
      case FEEDING:
        deploy.requestPosition(0.08962744474411011);
        if (requestFEEDINGS) {
          state = Superstates.FEEDINGS;
        }
        else if (requestIdle) {           
            state = Superstates.IDLE;
        }
        break;
        case FEEDINGS:
        deploy.requestPosition(0.5);
        if (requestFeed) {           
            state = Superstates.FEEDING;
        }
        else if (requestIdle) {           
          state = Superstates.IDLE;
      }
        break; 
        case ELEVATE:
        elevator.requestPosition(0.05);
        if (requestElevate){
          state = Superstates.ELEVATE;
        }
        else if (requestFeed) {           
          state = Superstates.FEEDING;
      }
      else if (requestIdle) {           
        state = Superstates.IDLE;
    }
    else if (requestFEEDINGS) {           
      state = Superstates.FEEDINGS;
  }
        break;
        case ELEVATE_IDLE:
        elevator.requestPosition(0.0001);
        if (requestElevateIdle){
          state = Superstates.ELEVATE_IDLE;
        }
        else if (requestFeed) {           
          state = Superstates.FEEDING;
      }
      else if (requestIdle) {           
        state = Superstates.IDLE;
    }
    else if (requestFEEDINGS) {           
      state = Superstates.FEEDINGS;
  }
        break;
    }
  }

  public Superstates getState() {
    return state;
  }

  public void requestIdle() {
    unsetAllRequests();
    requestIdle = true;
  }

  public void requestFeed() {
    unsetAllRequests();
    requestFeed = true;
  }

  public void requestFEEDINGS() {
    unsetAllRequests();
    requestFEEDINGS = true;
  }

  public void requestElevate() {
    unsetAllRequests();
    requestElevate = true;
  }

  public void requestElevateIdle() {
    unsetAllRequests();
    requestElevateIdle = true;
  }

  private void unsetAllRequests() {
    requestIdle = false;
    requestFeed = false;
    requestElevate = false;
    requestElevateIdle = false;
    requestFEEDINGS = false;
  }

}