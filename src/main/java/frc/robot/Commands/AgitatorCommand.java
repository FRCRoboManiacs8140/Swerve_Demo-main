// package frc.robot.Commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.IntakeSubsystem;
// import frc.robot.subsystems.AgitatorSubsystem;

// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase;



// public class AgitatorCommand extends Command {
    
//   // Instantiate Stuff
// public AgitatorSubsystem m_robotAgitate;
    

// public AgitatorCommand(AgitatorSubsystem robotAgitate) {
//     m_robotAgitate = robotAgitate;
// }

// @Override
// public void initialize() {

    
// }


// @Override
// public void execute() {
//     m_robotAgitate.agitate(DriveConstants.kAgitatorSpeed);
// }

// @Override
// public void end(boolean interrupted) {
//     m_robotAgitate.agitate(0);
// }

// @Override
// public boolean isFinished() {
//     m_robotAgitate.agitate(0);
//     return true;
// }
// }