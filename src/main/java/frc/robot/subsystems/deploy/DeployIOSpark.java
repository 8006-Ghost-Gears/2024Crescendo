package frc.robot.subsystems.deploy;

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

public class DeployIOSpark implements DeployIO {
  private SparkBase deploy;
  private RelativeEncoder encoder;

  private SparkMaxConfig deployConfig;

  private double targetPosition;

  // private ClosedLoopSlot slot2;

  public DeployIOSpark() {
    deploy = new SparkMax(Constants.Algae.deploy, MotorType.kBrushless);

    deployConfig = new SparkMaxConfig();

    configureDeploy(deploy, deployConfig);
  }

  private void configureDeploy(SparkBase motor, SparkBaseConfig config) {

    encoder = motor.getEncoder();
    encoder.setPosition(0);

    // config.disableFollowerMode();
    config.idleMode(IdleMode.kBrake);
    config.inverted(false);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.limitSwitch.forwardLimitSwitchEnabled(false);
    config.smartCurrentLimit(Constants.Algae.supplyCurrentLimit);
    config.closedLoop.pid(7, 0, 4);
    config.closedLoop.outputRange(Constants.Algae.peakReverse, Constants.Algae.peakForward);

    motor.configure(config, null, null);
  }

  @Override
  public void updateInputs(DeployIOInputs inputs) {
    inputs.pos = deploy.getEncoder().getPosition();
    inputs.appliedVoltage = deploy.getBusVoltage();
    inputs.supplyCurrentAmps = deploy.getOutputCurrent();
    inputs.tempCelsius = new double[] {deploy.getMotorTemperature(), deploy.getMotorTemperature()};
  }

  @Override
  public void setPosition(double targetPosition) {
    deploy.getClosedLoopController().setReference(targetPosition, ControlType.kPosition);
    // leader.setVoltage(heightMeters);
    Logger.recordOutput("Deploy/TargetPosition", targetPosition);
  }

  @Override
  public void setVoltage(double voltage) {
    deploy.setVoltage(voltage);
  }

  @Override
  public void seedPosition(double motorPositionRot) {
    encoder.setPosition(motorPositionRot);
  }

  @Override
  public void stop() {
    deploy.stopMotor();
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    deployConfig.idleMode(IdleMode.kBrake);
  }
}
