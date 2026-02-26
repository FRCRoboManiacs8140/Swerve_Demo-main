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
  public ShooterSubsystem m_robotShoot;
    // double m_distance;
    // double m_initialDistance;
//   SparkMax m_shooterFollowerRightMotor = new SparkMax(D, MotorType.kBrushless);
//   SparkMax m_shooterLeaderLeftMotor = new SparkMax(3, MotorType.kBrushless); 
  
  

public ShootCommand(ShooterSubsystem robotShoot) {
    m_robotShoot = robotShoot;
    // m_shooterFollowerRightMotor = m_shooterFollowerRightMotor; 
    // m_shooterLeaderLeftMotor = m_shooterLeaderLeftMotor;
    // m_robotDrive = robotDrive;
    // m_distance = distance;
    // addRequirements(m_robotDrive);
}

@Override
public void initialize() {

    
}


@Override
public void execute() {
    // m_robotShoot.set(DriveConstants.kShooterSpeed); ???
    // m_shooterFollowerRightMotor.set(DriveConstants.kShooterSpeed); 
    // m_shooterLeaderLeftMotor.set(DriveConstants.kShooterSpeed);
}

@Override
public void end(boolean interrupted) {
    // m_shooterFollowerRightMotor.set(0); 
    // m_shooterLeaderLeftMotor.set(0);
}

@Override
public boolean isFinished() {
    // m_shooterFollowerRightMotor.set(0); 
    // m_shooterLeaderLeftMotor.set(0);
    return true;
}
}