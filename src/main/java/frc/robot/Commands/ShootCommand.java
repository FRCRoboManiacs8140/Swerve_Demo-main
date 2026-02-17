package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShootCommand extends Command {
    
  // Instantiate Stuff
  ShooterSubsystem m_shooterSubsystem;
    double shooterSpeed = kShooterSpeed;
    
public ShootCommand(ShooterSubsystem shooterSubsystem) {
    m_shooterSubsystem = shooterSubsystem;
    addRequirements(m_shooterSubsystem);
}

@Override
public void execute() {
    shooterFollowerMotor.set(shooterSpeed);
    shooterLeaderMotor.set(shooterSpeed);
}

@Override
public void end(boolean interrupted) {
    shooterLeadermotor.set(0);
    shooterFollowerMotor.set(0);
}

@Override
public boolean isFinished() {
    shooterLeadermotor.set(0);
    shooterFollowerMotor.set(0);
}
}

