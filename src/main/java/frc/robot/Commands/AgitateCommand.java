package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.AgitatorSubsystem;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;



public class AgitateCommand extends Command {
    
  // Instantiate Stuff
public AgitatorSubsystem m_robotAgitate;
    
// Make the Agitate Command
public AgitateCommand(AgitatorSubsystem robotAgitate) {
    m_robotAgitate = robotAgitate;
}

// Run the Agitate Command
@Override
public void execute() {
    m_robotAgitate.agitate(DriveConstants.kAgitatorSpeed);
}

// If command is interrupted or ends, stop the agitator
@Override
public void end(boolean interrupted) {
    m_robotAgitate.stop();
}

@Override
public boolean isFinished() {
    m_robotAgitate.stop();
    return true;
}
}