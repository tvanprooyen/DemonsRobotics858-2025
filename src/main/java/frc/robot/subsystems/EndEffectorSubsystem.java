// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MechanismConstants;

public class EndEffectorSubsystem extends SubsystemBase {
  private SparkFlex tiltMotor;
  private SparkMax topFeedMotor;
  private SparkMax bottomFeedMotor;
  private RelativeEncoder tiltEncoder;
  private PIDController tiltPID;
  private double desiredAngle;

  public EndEffectorSubsystem() {
    tiltMotor = new SparkFlex(MechanismConstants.TILT_MOTOR_ID, MotorType.kBrushless);
    topFeedMotor = new SparkMax(MechanismConstants.TOP_FEED_MOTOR_ID, MotorType.kBrushless);
    bottomFeedMotor = new SparkMax(MechanismConstants.BOTTOM_FEED_MOTOR_ID, MotorType.kBrushless);

    tiltEncoder = tiltMotor.getEncoder();

    tiltEncoder.setPosition(0);

    tiltPID = new PIDController(0.12, 0.0, 0);

  }

  public void RunTopWheel(double speed){
     topFeedMotor.set(speed);
  }
  
  public void setBothWheels(double speed){
     topFeedMotor.set(speed);
     bottomFeedMotor.set(-speed);
  }

   public void TiltEndeffector(boolean Ontilt){
       if(Ontilt){
         desiredAngle = 3.391; // 50 degrees in rotations
       } else {
         desiredAngle = 0;
       }
   }

  @Override
  public void periodic() {

    double SetTiltAngle = tiltPID.calculate(tiltEncoder.getPosition(), desiredAngle);

    if(desiredAngle == 0) {
      SetTiltAngle *= 0.3;
    }

    tiltMotor.set(SetTiltAngle);

    //SmartDashboard.putNumber("Tilt Encoder", tiltEncoder.getPosition());
  }
}
