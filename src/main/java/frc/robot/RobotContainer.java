// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PS4Controller.Button;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Commands.AimCommand;
import frc.robot.Commands.StrafeCommand;
import frc.robot.Commands.ShootCommand;
import frc.robot.Commands.IntakeCommand;
import frc.robot.Commands.AgitateCommand;
import frc.robot.Commands.IndexCommand;
import frc.robot.Commands.DeployActuatorCommand;
import frc.robot.Commands.RetractActuatorCommand;
import frc.robot.Commands.Drive20Feet;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.AgitatorSubsystem;
import frc.robot.subsystems.IndexSubsystem;
import frc.robot.subsystems.ActuatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import java.util.List;
import edu.wpi.first.wpilibj2.command.InstantCommand; 
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.wpilibj2.command.button.POVButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();

  public DriveSubsystem getRobotDrive(){
    return m_robotDrive;

  }

  private final ShooterSubsystem m_robotShoot = new ShooterSubsystem();

  public ShooterSubsystem getRobotShooter(){ 
    return m_robotShoot;
  }

  private final IntakeSubsystem m_robotIntake = new IntakeSubsystem();

  public IntakeSubsystem getRobotIntake(){ 
    return m_robotIntake;
  }

  private final AgitatorSubsystem m_robotAgitate = new AgitatorSubsystem();
   
  public AgitatorSubsystem getRobotAgitate(){ 
    return m_robotAgitate;
  }

  private final IndexSubsystem m_robotIndex = new IndexSubsystem();

  public IndexSubsystem getRobotIndex(){ 
    return m_robotIndex;
  }
  
  private final ActuatorSubsystem m_robotActuate = new ActuatorSubsystem();

  public ActuatorSubsystem getRobotActuate(){ 
    return m_robotActuate;
  }

  // The driver's controller
  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));


    m_robotShoot.setDefaultCommand(
      new RunCommand(
        () -> m_robotShoot.shoot(0.5),
        m_robotShoot));

    m_robotAgitate.setDefaultCommand(
      new RunCommand(
        () -> m_robotAgitate.agitate(0.5),
        m_robotAgitate));

    m_robotIndex.setDefaultCommand(
      new RunCommand(
        () -> m_robotIndex.index(0.5),
        m_robotIndex));

    m_robotActuate.setDefaultCommand(
      new RunCommand(
        () -> m_robotActuate.deploy(0.5),
        m_robotActuate));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    // Map the setX command to the X button
    new JoystickButton(m_driverController, XboxController.Button.kX.value)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));  
     
    // Map the zeroHeading command to the A button        
    new JoystickButton(m_driverController, XboxController.Button.kA.value)
        .whileTrue(new RunCommand(
             () -> m_robotDrive.zeroHeading(),
            m_robotDrive));
    
    // Map the AimCommand to the B button
    new JoystickButton(m_driverController, XboxController.Button.kB.value)
    .onTrue(
      new AimCommand(m_robotDrive)
    );

    // Map the ShootCommand to the right trigger button
    new JoystickButton(m_driverController, XboxController.Axis.kRightTrigger.value)
    .onTrue(
      new ShootCommand(m_robotShoot)
    );

    // Map the IntakeCommand to the right bumper button
    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value)
    .onTrue(
      new IntakeCommand(m_robotIntake)
    );

    // Map the AgitateCommand to the left bumper button
    new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value)
    .onTrue(
      new AgitateCommand(m_robotAgitate)
    );

    // Map the IndexCommand to the left trigger button
    new JoystickButton(m_driverController, XboxController.Axis.kLeftTrigger.value)
    .onTrue(
      new IndexCommand(m_robotIndex)
    );

    // Map the DeployActuatorCommand to the D-pad up button
    new POVButton(m_driverController, 0) // 0 degrees for D-pad up
    .onTrue(
      new DeployActuatorCommand(m_robotActuate, DriveConstants.kActuatorExtendedPosition)
    );

    // Map the RetractActuatorCommand to the D-pad down button
    new POVButton(m_driverController, 180) // 180 degrees for D-pad down
    .onTrue(
      new RetractActuatorCommand(m_robotActuate, DriveConstants.kActuatorRetractedPosition)
    );
  
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new PathPlannerAuto("Zig Zag Auto");
  }
}
