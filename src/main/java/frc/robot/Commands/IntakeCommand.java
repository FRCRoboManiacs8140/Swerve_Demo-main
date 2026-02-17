package frc.robot.Commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command {
    
  // Instantiate Stuff
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
}

@Override
public boolean isFinished() {
    intakeMotor.set(0);
}
}

