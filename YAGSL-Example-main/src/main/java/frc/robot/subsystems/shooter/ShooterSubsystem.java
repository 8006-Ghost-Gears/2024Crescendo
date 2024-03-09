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
 CANSparkMax bigLeft;
 CANSparkMax littleLeft;
 CANSparkMax bigRight;
 CANSparkMax littleRight;
private final PIDController pidController = new PIDController(0.1, 0, 0);
  
  public ShooterSubsystem() {
    bigLeft = new CANSparkMax(CAN.bigLeft,MotorType.kBrushless);
    littleLeft = new CANSparkMax(CAN.littleLeft,MotorType.kBrushless);
    bigRight = new CANSparkMax(CAN.bigRight,MotorType.kBrushless);
    littleRight = new CANSparkMax(CAN.littleRight,MotorType.kBrushless);
    pivot = new CANSparkMax(CAN.pivot,MotorType.kBrushless);
    
    bigLeft.restoreFactoryDefaults();
    bigLeft.setInverted(false);
    
    littleLeft.restoreFactoryDefaults();
    littleLeft.setInverted(false);

    bigRight.restoreFactoryDefaults();
    bigRight.setInverted(true);
    
    littleRight.restoreFactoryDefaults();
    littleRight.setInverted(true);
    
    pivot.restoreFactoryDefaults();
    pivot.setInverted(true);
    pivot.getEncoder().setPosition(0);

  }


// sets shooter speed
  public void Shooter(double speed) 
  {
      bigLeft.set(speed);
      littleLeft.set(speed);
      bigRight.set(speed);
      littleRight.set(speed);
    }

    public void ShooterIntake(double speed)
    {
      bigLeft.set(-speed);
      littleLeft.set(-speed);
      bigRight.set(-speed);
      littleRight.set(-speed);

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
  bigLeft.set(0);
  littleLeft.set(0);
  bigRight.set(0);
  littleRight.set(0);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("shooterLeft", bigLeft.getEncoder().getPosition());
    SmartDashboard.putNumber("shooterRight", littleLeft.getEncoder().getPosition());
    SmartDashboard.putNumber("shooterLeft", bigRight.getEncoder().getPosition());
    SmartDashboard.putNumber("shooterRight", littleRight.getEncoder().getPosition());
    SmartDashboard.putNumber("pivot", pivot.getEncoder().getPosition());
  }
}