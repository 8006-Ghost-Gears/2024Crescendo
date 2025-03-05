// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.deploy.Deploy;
import frc.robot.subsystems.deploy.DeployIO;
import frc.robot.subsystems.deploy.DeployIOSpark;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.ElevatorIO;
import frc.robot.subsystems.elevator.ElevatorIOSpark;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandPS4Controller driver = new CommandPS4Controller(0);

  public static DeployIO deployIO =
  Constants.deployEnabled ? new DeployIOSpark() : new DeployIO() {};
  public static ElevatorIO elevatorIO =
  Constants.elevatorEnabled ? new ElevatorIOSpark() : new ElevatorIO() {};

  public static Deploy deploy = new Deploy(deployIO);
  public static Elevator elevator = new Elevator(elevatorIO);
  public static Superstructure superstructure =
      new Superstructure(deploy, elevator);
  // The robot's subsystems and commands are defined here...
<<<<<<< Updated upstream
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve/neo"));

  public static ClimberSubsystem m_ClimberSubsystem = new ClimberSubsystem();

  public static IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();

  public static ShooterSubsystem m_ShooterSubsystem = new ShooterSubsystem();

  public static Object m_ShakerSubsystem;

  private final SendableChooser<Command> autoChooser;

  public static final PS4Controller driver = new PS4Controller(DriveTeamConstants.driver);

  public static final PS4Controller operator = new PS4Controller(DriveTeamConstants.operator);

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
  JoystickButton IntakeShooter = new JoystickButton(operator, 7);
  JoystickButton Shooter = new JoystickButton(operator, 8);
  JoystickButton ShooterAmp = new JoystickButton(operator,1);
  // CLIMBER BUTTONS
  POVButton ClimberUpPosition = new POVButton(tester, 0);
  POVButton ClimberDownPosition = new POVButton(tester, 180);
  // CLIMBER UP AND DOWN ON TESTER CONTROLLER
  POVButton ClimberUp = new POVButton(operator, 0);
  POVButton ClimberDown = new POVButton(operator, 180);
  POVButton ClimberStringsDown = new POVButton(operator, 90);
=======
  private final SwerveSubsystem drivebase =
      new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve/neo"));
>>>>>>> Stashed changes

  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular
   * velocity.
   */
  SwerveInputStream driveAngularVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> driver.getLeftY() * -1,
              () -> driver.getLeftX() * -1)
          .withControllerRotationAxis(driver::getRightX)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  /** Clone's the angular velocity input stream and converts it to a fieldRelative input stream. */
  SwerveInputStream driveDirectAngle =
      driveAngularVelocity
          .copy()
          .withControllerHeadingAxis(driver::getRightX, driver::getRightY)
          .headingWhile(true);

  /** Clone's the angular velocity input stream and converts it to a robotRelative input stream. */
  SwerveInputStream driveRobotOriented =
      driveAngularVelocity.copy().robotRelative(true).allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(), () -> -driver.getLeftY(), () -> -driver.getLeftX())
          .withControllerRotationAxis(() -> driver.getRawAxis(2))
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard =
      driveAngularVelocityKeyboard
          .copy()
          .withControllerHeadingAxis(
              () -> Math.sin(driver.getRawAxis(2) * Math.PI) * (Math.PI * 2),
              () -> Math.cos(driver.getRawAxis(2) * Math.PI) * (Math.PI * 2))
          .headingWhile(true)
          .translationHeadingOffset(true)
          .translationHeadingOffset(Rotation2d.fromDegrees(0));

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));

    UsbCamera camera = CameraServer.startAutomaticCapture();

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedDirectAngleKeyboard =
        drivebase.driveFieldOriented(driveDirectAngleKeyboard);

    if (RobotBase.isSimulation()) {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity);
    }

    if (Robot.isSimulation()) {
      Pose2d target = new Pose2d(new Translation2d(1, 4), Rotation2d.fromDegrees(90));
      // drivebase.getSwerveDrive().field.getObject("targetPose").setPose(target);
      driveDirectAngleKeyboard.driveToPose(
          () -> target,
          new ProfiledPIDController(5, 0, 0, new Constraints(5, 2)),
          new ProfiledPIDController(
              5, 0, 0, new Constraints(Units.degreesToRadians(360), Units.degreesToRadians(180))));
      driver
          .options()
          .onTrue(
              Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driver.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driver
          .button(2)
          .whileTrue(
              Commands.runEnd(
                  () -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                  () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

      //      driverXbox.b().whileTrue(
      //          drivebase.driveToPose(
      //              new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
      //                              );

    }
    else{
      driver.cross().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driver.square().onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
      driver
          .circle()
          .whileTrue(
              drivebase.driveToPose(
                  new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0))));
      driver.options().whileTrue(Commands.none());
      driver.share().whileTrue(Commands.none());
      driver.R1().onTrue(new InstantCommand(
                () -> {
                  superstructure.requestFeed();
                })
            .ignoringDisable(true));
    }

    driver.L1().onTrue(new InstantCommand(
      () -> {
        superstructure.requestIdle();
      })
  .ignoringDisable(true));
    
  driver.R2().onTrue(new InstantCommand(
      () -> {
        superstructure.requestFEEDINGS();
      })
  .ignoringDisable(true));
    }     

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return drivebase.getAutonomousCommand("New Auto");
  }

  public void setMotorBrake(boolean brake) {
    drivebase.setMotorBrake(brake);
  }
}
