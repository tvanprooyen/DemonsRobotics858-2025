// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(19.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(19.5);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 9; //3;
    public static final int kRearLeftDrivingCanId =  3;  // 9;
    public static final int kFrontRightDrivingCanId = 7; // 5;
    public static final int kRearRightDrivingCanId = 5;// 7;

    public static final int kFrontLeftTurningCanId = 8; // 2;
    public static final int kRearLeftTurningCanId = 2; // 8;
    public static final int kFrontRightTurningCanId = 6; // 4;
    public static final int kRearRightTurningCanId = 4;// 6;

    public static final boolean kGyroReversed = false;
    public static final int GyroID = 12;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.1;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = 5.36; // (14.0 / 50.0) * (28.0/16.0) * (15.0/45.0); //(45.0 * 22) / (kDrivingMotorPinionTeeth * 15); //TODO Place correct gear ratio here
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.08; //TODO Correct Deadband
    public static final double kDriveRotDeadband = 0.085; //TODO Correct Deadband
    public static final int kMonkeyControllerPort = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class MechanismConstants{

    public static final int ELEVATOR_MOTOR_ID = 11;
    public static final int ELEVATOR_MOTOR_2_ID = 10;

    public static final int FEED_MOTOR_LEFT_ID = 21;
    public static final int FEED_MOTOR_RIGHT_ID = 20;
    public static final int INTAKE_MOTOR_ID = 25;

    public static final int TOP_FEED_MOTOR_ID = 23;
    public static final int BOTTOM_FEED_MOTOR_ID = 22;
    
    public static final int TILT_MOTOR_ID = 24;

    public static final double ELEVATOR_INTAKE_HIGHT = 1.3;
    public static final double ELEVATOR_CORAL_L2_HIGHT = 1.2;
    public static final double ELEVATOR_CORAL_L3_HIGHT = 2.9;
    public static final double ELEVATOR_CORAL_L4_HIGHT = 5.8;

    public static final double ELEVATOR_ALGAE_L2_HIGHT = 2.6;
    public static final double ELEVATOR_ALGAE_L3_HIGHT = 4.6;
    public static final double ELEVATORR_BARGE = 9.4;

    public static final double INTAKE_ARM_UP = 0.62;
    public static final double INTAKE_ARM_DOWN = 0.04;
    public static final double INTAKE_ARM_MID = 0.35;

  }
}
