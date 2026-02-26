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
<<<<<<< HEAD
  public SparkMax m_shooterLeaderLeftMotor;
  public SparkMax m_shooterFollowerRightMotor;
    // double m_initialDistance = m_robotShoot.getPose();
    // double m_distance = 180; // Target distance
    
// Make the shoot command
=======
    

>>>>>>> 7700cc8 (Build Succeeds, successfully implemented ShootCommand ShooterSubsystem and MAXSwerveSHooterModule)
public ShootCommand(ShooterSubsystem robotShoot) {
    m_robotShoot = robotShoot;
}

// Run the shoot command
@Override
public void execute() {
<<<<<<< HEAD
    // This uses the speed set in Constants
=======
>>>>>>> 7700cc8 (Build Succeeds, successfully implemented ShootCommand ShooterSubsystem and MAXSwerveSHooterModule)
    m_robotShoot.shoot(DriveConstants.kShooterSpeed);
}

// If command is interrupted or ends, stop the shooter
@Override
public void end(boolean interrupted) {
<<<<<<< HEAD
    m_robotShoot.stop();
=======
    m_robotShoot.shoot(0);
>>>>>>> 7700cc8 (Build Succeeds, successfully implemented ShootCommand ShooterSubsystem and MAXSwerveSHooterModule)
}

@Override
public boolean isFinished() {
<<<<<<< HEAD
=======
    m_robotShoot.shoot(0);
>>>>>>> 7700cc8 (Build Succeeds, successfully implemented ShootCommand ShooterSubsystem and MAXSwerveSHooterModule)
    return true;
    // if (m_robotShoot.getPose() - m_initialDistance >= m_distance) {
    //     return true;
    // } else {
    //     return false;
    // }
}
}
