package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

public class Drive20Feet extends Command {
    
  // Instantiate Stuff
  DriveSubsystem m_robotDrive;
    double m_distance;
    double m_initialDistance;

public Drive20Feet(DriveSubsystem robotDrive, double distance) {
    m_robotDrive = robotDrive;
    m_distance = distance;
    addRequirements(m_robotDrive);
}

@Override
public void initialize() {
    m_initialDistance = m_robotDrive.getPose().getX();
}


@Override
public void execute() {
    m_robotDrive.drive(0.25,0,0, false);
}

@Override
public void end(boolean interrupted) {
    m_robotDrive.drive(0,0,0, false);
}

@Override
public boolean isFinished() {
    if (m_robotDrive.getPose().getX() - m_initialDistance >= m_distance) {
        return true;
    } else {
        return false;
    }
}
}
