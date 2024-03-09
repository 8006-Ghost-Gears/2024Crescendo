// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveTeamConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.intake.FeederIntakeCommand;
import frc.robot.commands.intake.FeederOuttakeCommand;
import frc.robot.commands.intake.IntakeOutCommand;
import frc.robot.commands.intake.IntakeOutManual;
import frc.robot.commands.intake.IntakeRetractedCommand;
import frc.robot.commands.intake.IntakeRetractedManual;
import frc.robot.commands.shooter.ShooterAmpCommand;
import frc.robot.commands.shooter.ShooterPivotBackManual;
import frc.robot.commands.shooter.ShooterPivotForwardManual;
import frc.robot.commands.shooter.ShooterShootCommand;
import frc.robot.commands.shooter.ShooterShootIntakeCommand;
import frc.robot.commands.climber.ClimberUpPosition;
import frc.robot.commands.climber.ClimberUp;
import frc.robot.commands.climber.ClimberDownPosition;
import frc.robot.commands.climber.ClimberDown;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDriveAdv;
import frc.robot.subsystems.Climber.ClimberSubsystem;
import frc.robot.subsystems.Intake.IntakeSubsystem;
import frc.robot.subsystems.shooter.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import javax.print.attribute.standard.JobKOctetsSupported;

import com.pathplanner.lib.auto.AutoBuilder;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  public static ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();

  public static IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

  public static ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final PS4Controller driver = new PS4Controller(DriveTeamConstants.driver);

  private final PS4Controller operator = new PS4Controller(DriveTeamConstants.operator);

  private final PS4Controller tester = new PS4Controller(DriveTeamConstants.tester);

  //LIMELIGHT BUTTONS
  JoystickButton DistanceShoot = new JoystickButton(driver, 6);

  // INTAKE BUTTONS
  JoystickButton Intake = new JoystickButton(operator, 5);
  JoystickButton Outtake = new JoystickButton(operator, 6);
  // RETRACTED OR OUT
  JoystickButton Retracted = new JoystickButton(operator, 12);
  JoystickButton Out = new JoystickButton(operator, 11);
  //shooter buttons
  JoystickButton Forward = new JoystickButton(operator, 4);
  JoystickButton back = new JoystickButton(operator, 2);
  JoystickButton IntakeShooter = new JoystickButton(operator, 7);
  JoystickButton Shooter = new JoystickButton(operator, 8);
  JoystickButton ShooterAmp = new JoystickButton(operator,1);
  // CLIMBER BUTTONS
  POVButton ClimberUpPosition = new POVButton(tester, 0);
  POVButton ClimberDownPosition = new POVButton(tester, 180);
  // CLIMBER UP AND DOWN ON TESTER CONTROLLER
  POVButton ClimberUp = new POVButton(operator, 0);
  POVButton ClimberDown = new POVButton(operator, 180);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

   /*
 * POV is the D-Pad, Example: POVButton handOpen = new POVButton(operator, 90);
 * The POV buttons are referred to by the angle. Up is 0, right is 90, down is 180, and left is 270.
 * buttonNumber 1 is Square on a PS4 Controller
 * buttonNumber 2 is X on a PS4 Controller
 * buttonNumber 3 is Circle on a PS4 Controller
 * buttonNumber 4 is Triangle on a PS4 Controller
 * buttonNumber 5 is L1 on a PS4 Controller
 * buttonNumber 6 is R1 on a PS4 Controller
 * buttonNumber 7 is L2 on a PS4 Controller
 * buttonNumber 8 is R2 on a PS4 Controller
 * buttonNumber 9 is SHARE on a PS4 Controller
 * buttonNumber 10 is OPTIONS on a PS4 Controller
 * buttonNumber 11 is L3 on a PS4 Controller
 * buttonNumber 12 is R3 on a PS4 Controller
 * buttonNumber 13 is the PlayStaion Button on a PS4 Controller
 * buttonNumber 14 is the Touchpad on a PS4 Controller
 * 
 * https://www.chiefdelphi.com/t/make-motor-move-at-a-specific-rpm/396774
 * Click this link if your trying to send a motor to a certain position
 * 
 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/controllers/index.html
 * Click this link if your trying to send a motor to a certain position
 * 
 * If you ever run into that problem like The Java Server Crashed 5 Times and will not be restarted, (worked for me)
 * then just delete all the WPILIB VS Codes and The Visual Studio Code that is on the pc, this is because Wpilib uses their own
 * VS Code when you install the latest version!
 */
  public RobotContainer()
  {
    // Configure the trigger bindings
    
    configureBindings();

    UsbCamera camera = CameraServer.startAutomaticCapture();
    camera.setResolution(240, 160); //Usually (640,320) // Will work at 160, 120
    camera.setFPS(30);
    
    

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedAnglularVelocity = drivebase.driveCommand(
        () -> -MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driver.getRightX());

    Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> -MathUtil.applyDeadband(driver.getLeftY(), OperatorConstants.LEFT_Y_DEADBAND),
        () -> -MathUtil.applyDeadband(driver.getLeftX(), OperatorConstants.LEFT_X_DEADBAND),
        () -> -driver.getRawAxis(2));

    drivebase.setDefaultCommand(
        !RobotBase.isSimulation() ? driveFieldOrientedAnglularVelocity : driveFieldOrientedDirectAngleSim);

        autoChooser = AutoBuilder.buildAutoChooser();
    Shuffleboard.getTab("Pre-Match").add("Auto Chooser", autoChooser);

  }

  /*// simple proportional turning control with Limelight.
  // "proportional control" is a control algorithm in which the output is proportional to the error.
  // in this case, we are going to return an angular velocity that is proportional to the 
  // "tx" value from the Limelight.
  double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .035;

    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    // convert to radians per second for our drive method
    targetingAngularVelocity *= drivebase.kMaxAngularSpeed;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }

  // simple proportional ranging control with Limelight's "ty" value
  // this works best if your Limelight's mount height and target mount height are different.
  // if your limelight and target are mounted at the same or similar heights, use "ta" (area) for target ranging rather than "ty"
  double limelight_range_proportional()
  {    
    double kP = .1;
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= drivebase.maximumSpeed;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }*/

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // FEEDER BUTTONS
    Intake.whileTrue(new FeederIntakeCommand());
    Outtake.whileTrue(new FeederOuttakeCommand());
    // INTAKE BUTTONS
    Retracted.whileTrue(new IntakeRetractedManual());
    Out.whileTrue(new IntakeOutManual());

    //Shooter Buttons
    Forward.whileTrue(new ShooterPivotForwardManual());
    back.whileTrue(new ShooterPivotBackManual());
    IntakeShooter.whileTrue(new ShooterShootIntakeCommand());
    Shooter.whileTrue(new ShooterShootCommand());
    ShooterAmp.whileTrue(new ShooterAmpCommand());

    // CLIMBER BUTTONS
    ClimberUpPosition.onTrue(new ClimberUpPosition());
    ClimberDownPosition.onTrue(new ClimberDownPosition());
    // CLIMBER TESTER BUTTONS
    ClimberUp.whileTrue(new ClimberUp());
    ClimberDown.whileTrue(new ClimberDown());


    
    

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}
