// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class ElevatorSubsystem extends SubsystemBase {
   private SparkFlex motor1;
  private SparkFlexConfig motor2Config;
  private SparkFlex motor2;
  private SparkFlexConfig motor1Config;
  private RelativeEncoder ExternalEncoder;
  private PIDController elevatorPID;
  private double desiredRotations; // always will be a positive value!

  private SlewRateLimiter elevatoRateLimiter;


  public ElevatorSubsystem() {
    motor1 = new SparkFlex(MechanismConstants.ELEVATOR_MOTOR_ID, MotorType.kBrushless);
    motor2 = new SparkFlex(MechanismConstants.ELEVATOR_MOTOR_2_ID, MotorType.kBrushless);
    ExternalEncoder = motor1.getExternalEncoder();
    ExternalEncoder.setPosition(0);
    elevatorPID = new PIDController(0.2, 0.0, 0.0); //0.2,0,0

    motor1Config = new SparkFlexConfig();
    motor1Config.inverted(true); //Up is the opposite way
    motor1.configure(motor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    motor2Config = new SparkFlexConfig();
    motor2Config.follow(motor1);
    motor2.configure(motor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatoRateLimiter = new SlewRateLimiter(0.8);
  }

  public void setRotations(double rotations){
    desiredRotations = rotations;
  }


  public double EncoderRotations() {
    return ExternalEncoder.getPosition() * -1;
  }


  @Override
  public void periodic() {

    double ElevatorMotorSpeed = elevatorPID.calculate(EncoderRotations(), desiredRotations);

    if(ElevatorMotorSpeed > 0.4) {
      ElevatorMotorSpeed = 0.4; //0.4
    }
    if(ElevatorMotorSpeed < 0.0) { //made sure the elevator goes down slowly.
      ElevatorMotorSpeed *= 0.2;
    }

    motor1.set(elevatoRateLimiter.calculate(ElevatorMotorSpeed));
    // SmartDashboard.putNumber("elevator speed", ElevatorMotorSpeed);
    // SmartDashboard.putNumber("Elevator Position", EncoderRotations());
    // SmartDashboard.putNumber("Elevator Set Position", desiredRotations);
  }
}
