package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
// import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSpark implements ElevatorIO {
  private SparkBase elevator;
  private RelativeEncoder encoder;

  private SparkMaxConfig elevatorConfig;

  private double targetPosition;

  // private ClosedLoopSlot slot2;

  public ElevatorIOSpark() {
    elevator = new SparkMax(Constants.Elevator.elevator, MotorType.kBrushless);

    elevatorConfig = new SparkMaxConfig();

    configureElevator(elevator, elevatorConfig);
  }

  private void configureElevator(SparkBase motor, SparkBaseConfig config) {

    encoder = motor.getEncoder();
    encoder.setPosition(0);

    // config.disableFollowerMode();
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.smartCurrentLimit(Constants.Algae.supplyCurrentLimit);
    config.closedLoop.pid(1.0, 0, 0);
    config.closedLoop.outputRange(Constants.Elevator.peakReverse, Constants.Elevator.peakForward);

    motor.configure(config, null, null);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.pos = elevator.getEncoder().getPosition();
    inputs.appliedVoltage = elevator.getBusVoltage();
    inputs.supplyCurrentAmps = elevator.getOutputCurrent();
    inputs.tempCelsius = new double[] {elevator.getMotorTemperature(), elevator.getMotorTemperature()};
  }

  @Override
  public void setPosition(double targetPosition) {
    elevator.getClosedLoopController().setReference(targetPosition, ControlType.kPosition);
    // leader.setVoltage(heightMeters);
    Logger.recordOutput("Elevator/TargetPosition", targetPosition);
  }

  @Override
  public void setVoltage(double voltage) {
    elevator.setVoltage(voltage);
  }

  @Override
  public void seedPosition(double motorPositionRot) {
    encoder.setPosition(motorPositionRot);
  }

  @Override
  public void stop() {
    elevator.stopMotor();
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    elevatorConfig.idleMode(IdleMode.kBrake);
  }
}
