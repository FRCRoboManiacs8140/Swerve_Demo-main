package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Limelight.LimelightHelpers;
import frc.robot.subsystems.DriveSubsystem;

public class StrafeCommand extends Command {
    
  // Instantiate Stuff
  DriveSubsystem m_robotDrive;

  // Timer for cancellation with failed robot adjustment
  Timer timer = new Timer();

  // PID Stuff!
  PIDController m_StrafeController = new PIDController(VisionConstants.p_rotate_to_tag, VisionConstants.i_rotate_to_tag, VisionConstants.d_rotate_to_tag);
  

  // Boolean for checking for "Tag In View" 
  boolean tag_in_view;

  // Constructor
  public StrafeCommand(DriveSubsystem driveSubsystem) {
        
    // Definitions and setting parameters are equal to members!
    m_robotDrive = driveSubsystem;
    addRequirements(driveSubsystem);
  }

  // What we do to set up the command
  public void initialize() {
    
  

    // Checks for TIV
    tag_in_view = LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME);

    // Timer Reset
    timer.start();
    timer.reset();
  }
    
  // The actual control!
  public void execute() {

    // If tags are in view, rotate at a speed proportional to the offset robot relative!
    if (tag_in_view) m_robotDrive.drive(0, -limelight_strafe_proportional(), 0, false);

    // Otherwise we tell it to quit
    else tag_in_view = false;
  }

  // Add stuff we do after to reset here (a.k.a tag filters)
  public void end(boolean interrupted) {}

  // Are we done yet? Finishes when threshold is reached or if no tag in view or if timer is reached
  public boolean isFinished() {
    return (LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME) < VisionConstants.rotate_to_tag_max 
      && LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME) > -VisionConstants.rotate_to_tag_min) 
      || !tag_in_view || timer.get() > 2;
  }

  // Method that returns a double for how fast the robot needs to turn, farther angle from the tag is a faster turn
  private double limelight_strafe_proportional() {
    m_StrafeController.enableContinuousInput(-40, 40);
    
    // Proportional multiplier on the X-Offset value
    double targetingStrafeVelocity = m_StrafeController.calculate((LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME)));

    // Multiply by -1 because robot is CCW Positive. Multiply by a reduction 
    // multiplier to reduce speed. Scale TX up with robot speed.
    targetingStrafeVelocity *= 0.1 * DriveConstants.kMaxAngularSpeed;

    // Hooray
    return targetingStrafeVelocity;
  }
}