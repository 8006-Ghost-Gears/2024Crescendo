// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Climber;

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

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new climberSub. */
 CANSparkMax leftClimber;
 CANSparkMax rightClimber;
 CANSparkMax pivotClimber;
private final PIDController pidController = new PIDController(0.1, 0, 0);
  
  public ClimberSubsystem() {
    leftClimber = new CANSparkMax(CAN.leftClimber,MotorType.kBrushless);
    rightClimber = new CANSparkMax(CAN.rightClimber,MotorType.kBrushless);
    pivotClimber = new CANSparkMax(CAN.pivotClimber,MotorType.kBrushless);
    
    pivotClimber.restoreFactoryDefaults();
    pivotClimber.setInverted(false);
    pivotClimber.getEncoder().setPosition(0);
    
    leftClimber.restoreFactoryDefaults();
    leftClimber.setInverted(false);
    leftClimber.getEncoder().setPosition(0);
    
    rightClimber.restoreFactoryDefaults();
    rightClimber.setInverted(true);
    rightClimber.getEncoder().setPosition(0);

  }

  public void pivotClimber(double speed) {
    pivotClimber.set(speed);

  }
  public void pivotUp()
  {
    final double kIntakeTolerance = 2.0;

    double targetPosition = 0.5;

    // introduce a delay of 2 seconds

        pidController.setSetpoint(targetPosition);

        pivotClimber.getPIDController().setP(0.1);
        pivotClimber.getPIDController().setI(0);
        pivotClimber.getPIDController().setD(0);

        pidController.setTolerance(kIntakeTolerance);

        pivotClimber.getPIDController().setOutputRange(-0.30, 0.30);

        pivotClimber.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);
  }
  public void pivotDown()
  {
    final double kIntakeTolerance = 2.0;

    double targetPosition = 0.5;

    // introduce a delay of 2 seconds

        pidController.setSetpoint(targetPosition);

        pivotClimber.getPIDController().setP(0.1);
        pivotClimber.getPIDController().setI(0);
        pivotClimber.getPIDController().setD(0);

        pidController.setTolerance(kIntakeTolerance);

        pivotClimber.getPIDController().setOutputRange(-0.30, 0.30);

        pivotClimber.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);

  }
  public void ClimberDown(double speed) {
    // Check if current position is greater than -1
    if (leftClimber.getEncoder().getPosition() > 0)
    {
      leftClimber.set(-speed);
      rightClimber.set(-speed);
    }
    else
    {
      leftClimber.set(speed);
      rightClimber.set(speed);
    }
  }
  public void ClimberUp(double speed) {
    if (leftClimber.getEncoder().getPosition() < 40.404354095458984)
    {
    leftClimber.set(-speed);
    rightClimber.set(-speed);
    }
    else
    {
      leftClimber.set(speed);
      rightClimber.set(speed);

    }
  }

  public void ClimberDown() {
    
    final double kClimberTolerance = 2.0;

    double targetPosition = 0.5;

    // introduce a delay of 2 seconds

        pidController.setSetpoint(targetPosition);

        leftClimber.getPIDController().setP(0.1);
        leftClimber.getPIDController().setI(0);
        leftClimber.getPIDController().setD(0);
        
        rightClimber.getPIDController().setP(0.1);
        rightClimber.getPIDController().setI(0);
        rightClimber.getPIDController().setD(0);

        pidController.setTolerance(kClimberTolerance);

        leftClimber.getPIDController().setOutputRange(-0.50, 0.50);
        rightClimber.getPIDController().setOutputRange(-0.50, 0.50);

        leftClimber.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);
        rightClimber.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);

    }


/* In this code, we first get the current position of the shoulder encoder and store it in 
the position variable. Then, we check the position and use the WaitThenRunCommand to schedule 
the appropriate action with a 1-second delay. The WaitThenRunCommand is a custom command that 
extends InstantCommand and takes a delay time (in seconds) and a Runnable as parameters. When 
it is scheduled, it waits for the specified delay time and then runs the Runnable. */ 
public void ClimberUp() {

  final double kClimberTolerance = 2.0;

  double targetPosition = -270; //TO-DO Find Position

  // introduce a delay of 2 seconds

      pidController.setSetpoint(targetPosition);

      leftClimber.getPIDController().setP(0.1);
      leftClimber.getPIDController().setI(0);
      leftClimber.getPIDController().setD(0);
      
      rightClimber.getPIDController().setP(0.1);
      rightClimber.getPIDController().setI(0);
      rightClimber.getPIDController().setD(0);

      pidController.setTolerance(kClimberTolerance);

      leftClimber.getPIDController().setOutputRange(-0.50, 0.50);
      rightClimber.getPIDController().setOutputRange(-0.50, 0.50);

      leftClimber.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);
      rightClimber.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);

  }

 

public void stopClimber(){
  leftClimber.set(0); // elijah allen is not the best button boy, but his sister thoooooooo ;) Jonah:)
  rightClimber.set(0);
}
public void stopPivot(){
  pivotClimber.set(0); // elijah allen is not the best button boy, but his sister thoooooooo ;) Jonah:)
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("ClimberLeft", leftClimber.getEncoder().getPosition());
    SmartDashboard.putNumber("ClimberRight", rightClimber.getEncoder().getPosition());

  }
}