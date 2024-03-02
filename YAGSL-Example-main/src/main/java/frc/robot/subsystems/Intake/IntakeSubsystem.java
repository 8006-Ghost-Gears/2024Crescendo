// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.WPIMathJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CAN;
import frc.robot.RobotContainer;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new climberSub. */
 CANSparkMax feeder;
 CANSparkMax intake;
private final PIDController pidController = new PIDController(0.1, 0, 0);
  
  public IntakeSubsystem() {
    feeder = new CANSparkMax(CAN.feeder,MotorType.kBrushless);
    intake = new CANSparkMax(CAN.intake,MotorType.kBrushless);
    
    feeder.restoreFactoryDefaults();
    feeder.setInverted(false);
    feeder.getEncoder().setPosition(0);
    
    intake.restoreFactoryDefaults();
    intake.setInverted(true);
    intake.getEncoder().setPosition(0);

  }



  public void Feeder(double speed) {
      feeder.set(speed);
    }

  public void Intake(double speed) {
    intake.set(speed);


  }

  public void IntakeRetracted() {
    
    final double kIntakeTolerance = 2.0;

    double targetPosition = 0.5;

    // introduce a delay of 2 seconds

        pidController.setSetpoint(targetPosition);

        intake.getPIDController().setP(0.1);
        intake.getPIDController().setI(0);
        intake.getPIDController().setD(0);

        pidController.setTolerance(kIntakeTolerance);

        intake.getPIDController().setOutputRange(-0.30, 0.30);

        intake.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);

    }


/* In this code, we first get the current position of the shoulder encoder and store it in 
the position variable. Then, we check the position and use the WaitThenRunCommand to schedule 
the appropriate action with a 1-second delay. The WaitThenRunCommand is a custom command that 
extends InstantCommand and takes a delay time (in seconds) and a Runnable as parameters. When 
it is scheduled, it waits for the specified delay time and then runs the Runnable. */ 
public void IntakeOut() {

  final double kIntakeTolerance = 2.0;

    double targetPosition = 0.5;

    // introduce a delay of 2 seconds

        pidController.setSetpoint(targetPosition);

        intake.getPIDController().setP(0.1);
        intake.getPIDController().setI(0);
        intake.getPIDController().setD(0);

        pidController.setTolerance(kIntakeTolerance);

        intake.getPIDController().setOutputRange(-0.30, 0.30);

        intake.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);

  }

 

public void stopIntake(){
  intake.set(0);
}

public void stopFeeder(){
  feeder.set(0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("Feeder", feeder.getEncoder().getPosition());
    SmartDashboard.putNumber("Intake", intake.getEncoder().getPosition());

  }
}