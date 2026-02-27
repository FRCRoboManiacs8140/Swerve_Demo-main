package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;



public class IntakeCommand extends Command {
    
  // Instantiate Stuff
  public IntakeSubsystem m_robotIntake;
    

public IntakeCommand(IntakeSubsystem robotIntake) {
    m_robotIntake = robotIntake;
}

@Override
public void initialize() {

    
}


@Override
public void execute() {
    m_robotIntake.intake(DriveConstants.kShooterSpeed);
}

@Override
public void end(boolean interrupted) {
    m_robotIntake.intake(0);
}

@Override
public boolean isFinished() {
    m_robotIntake.intake(0);
    return true;
}
}