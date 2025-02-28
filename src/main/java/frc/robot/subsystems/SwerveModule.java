// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Configs;

public class SwerveModule {
  private final SparkFlex m_drivingSpark;
  private final SparkFlex m_turningSpark;

  private final RelativeEncoder m_drivingEncoder;
  private final AbsoluteEncoder m_turningEncoder;

  private final SparkClosedLoopController m_drivingClosedLoopController;
  private final SparkClosedLoopController m_turningClosedLoopController;

  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());


  public SwerveModule(int drivingCANId, int turningCANId) {
    m_drivingSpark = new SparkFlex(drivingCANId, MotorType.kBrushless);
    m_turningSpark = new SparkFlex(turningCANId, MotorType.kBrushless);

    m_drivingEncoder = m_drivingSpark.getEncoder();
    m_turningEncoder = m_turningSpark.getAbsoluteEncoder();

    m_drivingClosedLoopController = m_drivingSpark.getClosedLoopController();
    m_turningClosedLoopController = m_turningSpark.getClosedLoopController();

    // Apply the respective configurations to the SPARKS. Reset parameters before
    // applying the configuration to bring the SPARK to a known good state. Persist
    // the settings to the SPARK to avoid losing them on a power cycle.
    m_drivingSpark.configure(Configs.FLEXSwerveModule.drivingConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);
    m_turningSpark.configure(Configs.FLEXSwerveModule.turningConfig, ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    m_desiredState.angle = new Rotation2d(m_turningEncoder.getPosition());
    m_drivingEncoder.setPosition(0);
  }

  // Returns the current state of the module.
  public SwerveModuleState getState() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModuleState(m_drivingEncoder.getVelocity(),
        new Rotation2d(m_turningEncoder.getPosition()));
  }

  // Returns the current position of the module.
  public SwerveModulePosition getPosition() {
    // Apply chassis angular offset to the encoder position to get the position
    // relative to the chassis.
    return new SwerveModulePosition(
        m_drivingEncoder.getPosition(),
        new Rotation2d(m_turningEncoder.getPosition()));
  }

  // Sets the desired state for the module.
  public void setDesiredState(SwerveModuleState desiredState) {

    // Optimize the reference state to avoid spinning further than 90 degrees.
    desiredState.optimize(new Rotation2d(m_turningEncoder.getPosition()));

    // Command driving and turning SPARKS towards their respective setpoints.
    m_drivingClosedLoopController.setReference(desiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turningClosedLoopController.setReference(desiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = desiredState;
  }

  /** Zeroes all the SwerveModule encoders. */
  public void resetEncoders() {
    m_drivingEncoder.setPosition(0);
  }
}
