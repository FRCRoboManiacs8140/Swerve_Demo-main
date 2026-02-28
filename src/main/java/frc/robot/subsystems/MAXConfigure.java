// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Configs;

public class MAXConfigure {
  // Create SparkMax object as well as its encoder and closed loop controller objects to be used in the subsystem. This
  private final SparkMax m_SparkMax;
  private final RelativeEncoder m_SparkMaxEncoder;
  private final SparkClosedLoopController m_SparkMaxClosedLoopController;
  // private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  /**
   * Constructs a MAXSwerveModule and configures the driving and turning motor,
   * encoder, and PID controller. This configuration is specific to the REV
   * MAXSwerve Module built with NEOs, SPARKS MAX, and a Through Bore
   * Encoder.
   */
  public MAXConfigure(int kSparkMaxCANId) {
    m_SparkMax = new SparkMax(kSparkMaxCANId, MotorType.kBrushless);

    m_SparkMaxEncoder = m_SparkMax.getEncoder();

    m_SparkMaxClosedLoopController = m_SparkMax.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    // m_SparkMax.configure(Configs.MAXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
    //     PersistMode.kPersistParameters);
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
      m_SparkMaxEncoder.getPosition(),
        new Rotation2d(m_SparkMaxEncoder.getPosition()));
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_SparkMaxEncoder.setPosition(0);
  }
}
