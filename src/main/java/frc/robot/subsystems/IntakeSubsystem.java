// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class IntakeSubsystem extends SubsystemBase {
  
  private SparkMax intakeMotor;
  private SparkMax feedMotorRight;
  private SparkMax feedMotorLeft;
  private SparkMaxConfig feedMotor2config;
  private SparkMaxConfig intakeMotorConfig;
  private AbsoluteEncoder intakeEncoder;
  private PIDController intakePID;
  private double desiredAngle;

  public IntakeSubsystem() {
     intakeMotor = new SparkMax(MechanismConstants.INTAKE_MOTOR_ID, MotorType.kBrushless);
     feedMotorRight = new SparkMax(MechanismConstants.FEED_MOTOR_RIGHT_ID, MotorType.kBrushless);
     feedMotorLeft = new SparkMax(MechanismConstants.FEED_MOTOR_LEFT_ID, MotorType.kBrushless);
     intakeEncoder = intakeMotor.getAbsoluteEncoder();
     intakePID = new PIDController(3, 0.0, 0.0);

     feedMotor2config = new SparkMaxConfig();
     feedMotor2config.follow(feedMotorRight, true);
     feedMotorLeft.configure(feedMotor2config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

     intakeMotorConfig = new SparkMaxConfig();
     intakeMotorConfig.smartCurrentLimit(80);

     intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

     this.desiredAngle = 0.62;
  }

  public void SetAngle(double desiredAngle){
     this.desiredAngle = desiredAngle; // from 0.04 to 0.43
  }

  public void SetFeedMotors(double speed){
    feedMotorRight.set(-speed);
  }

  @Override
  public void periodic() {
     intakeMotor.set(intakePID.calculate(intakeEncoder.getPosition(), this.desiredAngle));

     //SmartDashboard.putNumber("Intake Angle", intakeEncoder.getPosition());
  }
}
