package frc.robot.subsystems.shooter;

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

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new climberSub. */
 CANSparkMax pivot;
 CANSparkMax shooterLeftOne;
 CANSparkMax shooterLeftTwo;
 CANSparkMax shooterRightOne;
 CANSparkMax shooterRightTwo;
private final PIDController pidController = new PIDController(0.1, 0, 0);
  
  public ShooterSubsystem() {
    shooterLeftOne = new CANSparkMax(CAN.shooterLeftOne,MotorType.kBrushless);
    shooterLeftTwo = new CANSparkMax(CAN.shooterLeftTwo,MotorType.kBrushless);
    shooterRightOne = new CANSparkMax(CAN.shooterRightOne,MotorType.kBrushless);
    shooterRightTwo = new CANSparkMax(CAN.shooterRightTwo,MotorType.kBrushless);
    pivot = new CANSparkMax(CAN.pivot,MotorType.kBrushless);
    
    shooterLeftOne.restoreFactoryDefaults();
    shooterLeftOne.setInverted(false);
    
    shooterLeftTwo.restoreFactoryDefaults();
    shooterLeftTwo.setInverted(false);

    shooterRightOne.restoreFactoryDefaults();
    shooterRightOne.setInverted(true);
    
    shooterRightTwo.restoreFactoryDefaults();
    shooterRightTwo.setInverted(true);
    
    pivot.restoreFactoryDefaults();
    pivot.setInverted(true);
    pivot.getEncoder().setPosition(0);

  }


// sets shooter speed
  public void Shooter(double speed) 
  {
      shooterLeftOne.set(speed);
      shooterLeftTwo.set(speed);
      shooterRightOne.set(speed);
      shooterRightTwo.set(speed);
    }

    public void ShooterIntake(double speed)
    {
      shooterLeftOne.set(speed);
      shooterLeftTwo.set(speed);
      shooterRightOne.set(speed);
      shooterRightTwo.set(speed);

    }

//sets pivot speed
  public void pivot(double speed) {
    pivot.set(speed);


  }
//pivot pack angle
  public void pivotBack() {
    
    final double kIntakeTolerance = 2.0;

    double targetPosition = 0.5;

    // introduce a delay of 2 seconds

        pidController.setSetpoint(targetPosition);

        pivot.getPIDController().setP(0.1);
        pivot.getPIDController().setI(0);
        pivot.getPIDController().setD(0);

        pidController.setTolerance(kIntakeTolerance);

        pivot.getPIDController().setOutputRange(-0.30, 0.30);

        pivot.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);

    }


/* In this code, we first get the current position of the shoulder encoder and store it in 
the position variable. Then, we check the position and use the WaitThenRunCommand to schedule 
the appropriate action with a 1-second delay. The WaitThenRunCommand is a custom command that 
extends InstantCommand and takes a delay time (in seconds) and a Runnable as parameters. When 
it is scheduled, it waits for the specified delay time and then runs the Runnable. */ 
// pivot forward angle for shooting
public void pivotForward() {

  final double kIntakeTolerance = 2.0;

    double targetPosition = 0.5;

    // introduce a delay of 2 seconds

        pidController.setSetpoint(targetPosition);

        pivot.getPIDController().setP(0.1);
        pivot.getPIDController().setI(0);
        pivot.getPIDController().setD(0);

        pidController.setTolerance(kIntakeTolerance);

        pivot.getPIDController().setOutputRange(-0.30, 0.30);

        pivot.getPIDController().setReference(targetPosition, CANSparkMax.ControlType.kPosition);

  }

 //stop pivot

public void stopPivot(){
  pivot.set(0);
}
//stops shooter
public void stopShooter(){
  shooterLeftOne.set(0);
  shooterLeftTwo.set(0);
  shooterRightOne.set(0);
  shooterRightTwo.set(0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooterLeft", shooterLeftOne.getEncoder().getPosition());
    SmartDashboard.putNumber("shooterRight", shooterLeftTwo.getEncoder().getPosition());
    SmartDashboard.putNumber("shooterLeft", shooterRightOne.getEncoder().getPosition());
    SmartDashboard.putNumber("shooterRight", shooterRightTwo.getEncoder().getPosition());
    SmartDashboard.putNumber("pivot", pivot.getEncoder().getPosition());
  }
}