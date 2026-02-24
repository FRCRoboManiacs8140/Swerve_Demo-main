package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ShooterSubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;



public class ShootCommand extends Command {
    
  // Instantiate Stuff
  ShooterSubsystem m_shooterFollowerRightMotor;
  ShooterSubsystem m_shooterLeaderLeftMotor; 
  

// public ShootCommand(ShooterSubsystem m_shooterFollowerRightMotor, double kShooterSpeed) {
//     m_shooterFollowerRightMotor = shooterFollowerRightMotor; 
//     m_shooterLeaderLeftMotor = shooterLeaderLeftMotor;
//     // m_robotDrive = robotDrive;
//     // m_distance = distance;
//     // addRequirements(m_robotDrive);
// }

@Override
public void initialize() {
    
}


@Override
public void execute() {
    m_shooterFollowerRightMotor.set(DriveConstants.kShooterSpeed); 
    m_shooterLeaderLeftMotor.set(DriveConstants.kShooterSpeed);
}

@Override
public void end(boolean interrupted) {
    m_shooterFollowerRightMotor.set(0); 
    m_shooterLeaderLeftMotor.set(0);
}

@Override
public boolean isFinished() {
    m_shooterFollowerRightMotor.set(0); 
    m_shooterLeaderLeftMotor.set(0);
}
}
