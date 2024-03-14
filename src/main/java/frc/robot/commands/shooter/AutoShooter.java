// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class AutoShooter extends Command {
  /** Creates a new AutoShooter. */
  Timer timer = new Timer();
  public AutoShooter() {
    
    addRequirements(RobotContainer.m_ShooterSubsystem);
    addRequirements(RobotContainer.m_IntakeSubsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RobotContainer.m_ShooterSubsystem.Shooter(1);
    if (Timer.getFPGATimestamp() > 1)
    {
    RobotContainer.m_ShooterSubsystem.Shooter(1);
    RobotContainer.m_IntakeSubsystem.Feeder(-1);
    }  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    RobotContainer.m_ShooterSubsystem.stopShooter();
    RobotContainer.m_IntakeSubsystem.stopFeeder();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
