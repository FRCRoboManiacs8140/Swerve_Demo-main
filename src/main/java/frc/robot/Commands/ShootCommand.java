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
    

public ShootCommand(ShooterSubsystem robotShoot) {
    m_robotShoot = robotShoot;
}

@Override
public void initialize() {

    
}


@Override
public void execute() {
    m_robotShoot.shoot(DriveConstants.kShooterSpeed);
}

@Override
public void end(boolean interrupted) {
    m_robotShoot.stop();
}

@Override
public boolean isFinished() {
    m_robotShoot.stop();
    return true;
}
}