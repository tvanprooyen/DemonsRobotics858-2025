// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final SwerveModule m_frontLeft = new SwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId
      );

  private final SwerveModule m_frontRight = new SwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId
      );

  private final SwerveModule m_rearLeft = new SwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId
      );

  private final SwerveModule m_rearRight = new SwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId
      );

  // The gyro sensor
  //private final ADIS16470_IMU m_gyro = new ADIS16470_IMU();
  private final Pigeon2  m_gyro = new Pigeon2(DriveConstants.GyroID);

  private RobotConfig config;

  private double speedMultiple = 1;
  private final Field2d field = new Field2d();

  // Odometry class for tracking robot pose
  SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
      DriveConstants.kDriveKinematics,
      GyroRot2d(),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      });

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {

      try{
          config = RobotConfig.fromGUISettings();
      } catch (Exception e) {
          // Handle exception as needed
        e.fillInStackTrace();
        throw new RuntimeException("An error occurred while getting the config.", e);
      }

      AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  private Rotation2d GyroRot2d() {
    return m_gyro.getRotation2d();
  }

  public void invertGyro() {
    m_gyro.setYaw(m_gyro.getYaw().getValueAsDouble() - 180); //Takes original angle and inverts, for after auto
  }

  public void resetInvertGyro() {
    zeroHeading();
    invertGyro();
  }

  @Override
  public void periodic() {
    // Update the odometry in the periodic block
    m_odometry.update(
        GyroRot2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
            //new SwerveModulePosition(-m_rearRight.getPosition().distanceMeters, m_rearRight.getPosition().angle)
        });

        
        
        // SmartDashboard.putNumber("gyro", GyroRot2d().getDegrees());
        // SmartDashboard.putNumber("poseX", m_odometry.getPoseMeters().getX());
        // SmartDashboard.putNumber("poseY", m_odometry.getPoseMeters().getY());
        // SmartDashboard.putData("field", field);
        field.setRobotPose(m_odometry.getPoseMeters().getX(), m_odometry.getPoseMeters().getY(), m_odometry.getPoseMeters().getRotation());
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    // Convert the commanded speeds into the correct units for the drivetrain
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;
    ChassisSpeeds speeds =  new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered);

    if(fieldRelative){
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(m_gyro.getYaw().getValueAsDouble()));
    } 
    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond * speedMultiple);
    setModuleStates(swerveModuleStates);
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
    setModuleStates(moduleStates);
  }

  // Sets the wheels into an X formation to prevent movement.
  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  // Sets the swerve ModuleStates.
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(
        desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }
  // Returns the currently-estimated pose of the robot.
  public Pose2d getPose() {
    Pose2d pose = m_odometry.getPoseMeters();
    return pose;
  }

  // Resets the odometry to the specified pose.
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        GyroRot2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }
  
  public void zeroOdometry() {
    m_odometry.resetPosition(
        GyroRot2d(),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        new Pose2d(0, 0, GyroRot2d()));
  }

  /** Resets the drive encoders to currently read a position of 0. */
  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }


  public ChassisSpeeds getRobotRelativeSpeeds() {
     SwerveModuleState[] moduleStates = new SwerveModuleState[] {
        m_frontLeft.getState(),
        m_frontRight.getState(),
        m_rearLeft.getState(),
        m_rearRight.getState()
     };
     return DriveConstants.kDriveKinematics.toChassisSpeeds(moduleStates);
  }

  /** Zeroes the heading of the robot. */
  public void zeroHeading() {
     m_gyro.reset();
     zeroOdometryHeading();
  }

  public void zeroOdometryHeading() {
    m_odometry.resetPosition(
    GyroRot2d(),
    new SwerveModulePosition[] {
        m_frontLeft.getPosition(),
        m_frontRight.getPosition(),
        m_rearLeft.getPosition(),
        m_rearRight.getPosition()
    },
    new Pose2d(m_odometry.getPoseMeters().getX(), m_odometry.getPoseMeters().getY(), GyroRot2d()));
  }

  // Returns the heading of the robot.
  public double getHeading() {
    return GyroRot2d().getDegrees();
  }

  // Returns the turn rate of the robot.
  public double getTurnRate() {
    return GyroRot2d().getDegrees() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void setSpeed(double speedMultiple){
    this.speedMultiple = speedMultiple;
  }
}
