// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.swerve.SwerveDrive;

import java.io.File;
import java.util.ArrayList;
import java.util.List;

public class TrajectoryUtils {
  public static FollowPathHolonomic generatePPHolonomicCommand(
    SwerveDrive swerveDrive, String pathName, double maxSpeed) {
  PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

  return generatePPHolonomicCommand(swerveDrive, path, maxSpeed, false);
}

public static FollowPathHolonomic generatePPHolonomicCommand(
    SwerveDrive swerveDrive, String pathName, double maxSpeed, boolean manualFlip) {
  PathPlannerPath path = PathPlannerPath.fromPathFile(pathName);

  return generatePPHolonomicCommand(swerveDrive, path, maxSpeed, manualFlip);
}

public static FollowPathHolonomic generatePPHolonomicCommand(
    SwerveDrive swerveDrive, PathPlannerPath path, double maxSpeed) {

  return generatePPHolonomicCommand(swerveDrive, path, maxSpeed, false);
}

public static FollowPathHolonomic generatePPHolonomicCommand(
    SwerveDrive swerveDrive,
    PathPlannerPath path,
    double maxSpeed,
    boolean manualFlip) {
  return new FollowPathHolonomic(
      path,
      () -> swerveDrive::getPoseMeters,
      swerveDrive::getChassisSpeed,
      swerveDrive::setChassisSpeedControl,
      new HolonomicPathFollowerConfig(
          new PIDConstants(5.0, 0, 0),
          new PIDConstants(5.0, 0, 0),
          maxSpeed,
          0.86210458762,
          new ReplanningConfig(false, false, 1.0, 0.25)),
      () -> !manualFlip,
      swerveDrive);
}
}
/*} else {
      try {
        return PathPlannerPath.loadPathGroup(fileName, pathConstraint, segmentConstraints);
      } catch (Exception e) {
        DriverStation.reportError("TrajectoryUtils::readTrajectory failed for " + fileName, null);
        return new ArrayList<PathPlannerTrajectory>();
        // TODO: handle exception
      } */