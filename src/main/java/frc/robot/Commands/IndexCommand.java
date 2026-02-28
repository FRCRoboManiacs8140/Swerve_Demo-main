// package frc.robot.Commands;

// import edu.wpi.first.math.controller.PIDController;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.Constants.DriveConstants;
// import frc.robot.Constants.AutoConstants;
// import frc.robot.subsystems.IndexSubsystem;
// import com.revrobotics.spark.SparkMax;
// import com.revrobotics.spark.config.SparkMaxConfig;
// import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
// import com.revrobotics.spark.SparkLowLevel.MotorType;
// import com.revrobotics.RelativeEncoder;
// import com.revrobotics.spark.SparkBase;



// public class IndexCommand extends Command {
    
//   // Instantiate Stuff
// public IndexSubsystem m_robotIndex;
    
// // Make the Agitate Command
// public IndexCommand(IndexSubsystem robotIndex) {
//     m_robotIndex = robotIndex;
// }

// // Run the Agitate Command
// @Override
// public void execute() {
//     m_robotIndex.index(DriveConstants.kIndexSpeed);
// }

// // If command is interrupted or ends, stop the agitator
// @Override
// public void end(boolean interrupted) {
//     m_robotIndex.stop();
// }

// @Override
// public boolean isFinished() {
//     m_robotIndex.stop();
//     return false;
// }
// }