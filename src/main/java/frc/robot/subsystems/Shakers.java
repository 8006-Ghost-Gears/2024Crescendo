// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;

public class Shakers extends SubsystemBase {
  /** Creates a new Shakers. */
  public Shakers() {
    
  }

  public void maxRumble() {
    RobotContainer.driver.setRumble(GenericHID.RumbleType.kBothRumble, 1);
    
  
  }

  
  public void noRumble() {
    RobotContainer.driver.setRumble(GenericHID.RumbleType.kBothRumble, 0);
    
  
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
