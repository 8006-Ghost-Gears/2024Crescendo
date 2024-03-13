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
    
    bigLeft.restoreFactoryDefaults();
    bigLeft.setInverted(false);
    
    littleLeft.restoreFactoryDefaults();
    littleLeft.setInverted(false);

    bigRight.restoreFactoryDefaults();
    bigRight.setInverted(true);
    
    littleRight.restoreFactoryDefaults();
    littleRight.setInverted(true);

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
  }
}