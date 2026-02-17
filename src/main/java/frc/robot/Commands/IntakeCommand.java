package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
<<<<<<< HEAD
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
=======
import frc.robot.subsystems.IntakeSubsystem;
>>>>>>> 16f5770 (Saving Before making risky changes)

public class IntakeCommand extends Command {
    
  // Instantiate Stuff
<<<<<<< HEAD
  public IntakeSubsystem m_robotIntake;
    
// Make the Intake Command
public IntakeCommand(IntakeSubsystem robotIntake) {
    m_robotIntake = robotIntake;
}

// Run the Intake Command
@Override
public void execute() {
    // This uses the speed set in Constants
    m_robotIntake.intake(DriveConstants.kShooterSpeed);
}

// If command is interrupted or ends, stop the intake
@Override
public void end(boolean interrupted) {
    m_robotIntake.stop();
=======
  IntakeSubsystem m_intakeSubsystem;
    double intakeSpeed = kIntakeSpeed;
    
public IntakeCommand(IntakeSubsystem intakeSubsystem) {
    m_intakeSubsystem = intakeSubsystem;
    addRequirements(m_intakeSubsystem);
}

@Override
public void execute() {
    intakeMotor.set(0.5);
}

@Override
public void end(boolean interrupted) {
    intakeMotor.set(0);
>>>>>>> 16f5770 (Saving Before making risky changes)
}

@Override
public boolean isFinished() {
<<<<<<< HEAD
    m_robotIntake.stop();
    return true;
}
}
=======
    intakeMotor.set(0);
}
}

>>>>>>> 16f5770 (Saving Before making risky changes)
