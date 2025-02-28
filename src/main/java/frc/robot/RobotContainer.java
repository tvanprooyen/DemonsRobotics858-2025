// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Commands.CoralElevatorCommand;
import frc.robot.Commands.DeployAlgaeCommand;
import frc.robot.Commands.DeployCoralCommand;
import frc.robot.Commands.DeployCoralL4Command;
import frc.robot.Commands.IntakeAlgaeCommand;
import frc.robot.Commands.LowerElevatorAlgaeCommand;
import frc.robot.Commands.LowerElevatorCommand;
import frc.robot.Commands.MidIntakeAlgae;
import frc.robot.Commands.RetractntakeArmsCommand;
import frc.robot.Commands.ElevatorBargeCommand;
import frc.robot.Commands.SetElevatorCommand;
import frc.robot.Constants.MechanismConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem drivetrain = new DriveSubsystem();
  private final EndEffectorSubsystem endEffector = new EndEffectorSubsystem();
  private final IntakeSubsystem intake = new IntakeSubsystem();
  private final ElevatorSubsystem elevator = new ElevatorSubsystem();

  // The driver's controller
  CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  // The Button Monkey Controller
  CommandXboxController m_monkeyController = new CommandXboxController(OIConstants.kMonkeyControllerPort);

  //Autonomus Chooser
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {


    NamedCommands.registerCommand("CoralL2", new SetElevatorCommand(elevator, MechanismConstants.ELEVATOR_CORAL_L2_HIGHT).withTimeout(0.9));
    NamedCommands.registerCommand("CoralL3", new SetElevatorCommand(elevator, MechanismConstants.ELEVATOR_CORAL_L3_HIGHT).withTimeout(1.0));
    NamedCommands.registerCommand("CoralL4", new ParallelCommandGroup(new SetElevatorCommand(elevator, MechanismConstants.ELEVATOR_CORAL_L4_HIGHT), new DeployAlgaeCommand(endEffector, -0.25, false)));
    NamedCommands.registerCommand("AlgaeL2", new SetElevatorCommand(elevator, MechanismConstants.ELEVATOR_ALGAE_L2_HIGHT).withTimeout(0.5));
    NamedCommands.registerCommand("IntakeAlgea", new DeployAlgaeCommand(endEffector, 0.3, true).withTimeout(1.0));
    NamedCommands.registerCommand("DeployCoral",  new DeployCoralCommand(endEffector, 0.4));
    NamedCommands.registerCommand("LowerElevator", new LowerElevatorCommand(elevator, endEffector));
    NamedCommands.registerCommand("LowerAlgaeElevator", new LowerElevatorAlgaeCommand(elevator, intake));

    NamedCommands.registerCommand("DeployCoralL4", new DeployCoralL4Command(elevator, endEffector));
    
    NamedCommands.registerCommand("elevatorIntakeUp", new SetElevatorCommand(elevator, MechanismConstants.ELEVATOR_INTAKE_HIGHT).withTimeout(0.5));
    NamedCommands.registerCommand("IntakeCoral", new DeployAlgaeCommand(endEffector, -0.5, false).withTimeout(2.0));
    NamedCommands.registerCommand("elevatorIntakeUpDown", new CoralElevatorCommand(elevator, 2.5));

    //AutoDriveCommands
    NamedCommands.registerCommand("invertGyro", new InstantCommand(drivetrain::invertGyro, drivetrain)); //Sets Teleop in the correct direction


    autoChooser =  AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("autoChooser", autoChooser);

    // Configure the button bindings
    configureButtonBindings();

    // Configure drive commands
    drivetrain.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> drivetrain.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                  (m_driverController.rightBumper().getAsBoolean()) ? 
                    -0.05 : 
                    (m_driverController.leftBumper().getAsBoolean()) ? 
                    0.05 : 
                    -MathUtil.applyDeadband((m_driverController.getLeftX())
                  , 
                  OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveRotDeadband),
                !(m_driverController.rightBumper().getAsBoolean() || m_driverController.leftBumper().getAsBoolean())),
                drivetrain));
  }


  private void configureButtonBindings() {

    // reset gyro
    m_driverController.start().onTrue(new InstantCommand(drivetrain::zeroHeading, drivetrain));

    //Zero Odometery
    m_driverController.back().onTrue(new InstantCommand(drivetrain::zeroOdometry, drivetrain));

    // Intake algae from ground 
    m_driverController.rightTrigger().whileTrue(new IntakeAlgaeCommand(intake, endEffector, 0.4, 0.3)).onFalse(new InstantCommand(() -> {
      intake.SetFeedMotors(0.6);
      endEffector.setBothWheels(0.3);
      intake.SetAngle(0.62);
    }, intake, endEffector).withTimeout(3).andThen(new InstantCommand(() -> {
      intake.SetFeedMotors(0);
      endEffector.setBothWheels(0);
    }, intake, endEffector)));

    // Deploy Algae to Processor and intake Coral
    m_driverController.leftTrigger().whileTrue(new DeployAlgaeCommand(endEffector, -0.5, false));
    
    //Intake Algae from Reef
    m_monkeyController.rightBumper().whileTrue(new DeployAlgaeCommand(endEffector, 0.3, true)); //new IntakeAlgaeReefCommand(endEffector));

    //Set Elevator Algae
    m_monkeyController.leftBumper().and(m_monkeyController.a()).whileTrue(new SetElevatorCommand(elevator, MechanismConstants.ELEVATOR_ALGAE_L2_HIGHT)).onFalse(new LowerElevatorAlgaeCommand(elevator, intake)); //L2
    m_monkeyController.leftBumper().and(m_monkeyController.b()).whileTrue(new SetElevatorCommand(elevator, MechanismConstants.ELEVATOR_ALGAE_L3_HIGHT)).onFalse(new LowerElevatorAlgaeCommand(elevator, intake)); //L3
    m_monkeyController.leftBumper().and(m_monkeyController.y()).whileTrue(new ElevatorBargeCommand(intake, elevator)).onFalse(new SequentialCommandGroup(new DeployAlgaeCommand(endEffector, -0.5, true).withTimeout(1.0), new LowerElevatorAlgaeCommand(elevator, intake))); //Barge
    m_monkeyController.leftBumper().and(m_monkeyController.y()).whileTrue(new InstantCommand(()-> {drivetrain.setSpeed(0.06);}, drivetrain))
    .whileFalse(new InstantCommand(()-> {drivetrain.setSpeed(1.0);}, drivetrain));

    //Set Elevator Coral

    SequentialCommandGroup DeployCoral = new SequentialCommandGroup(new DeployCoralCommand(endEffector, 0.4), new LowerElevatorCommand(elevator, endEffector));

    m_monkeyController.leftBumper().negate().and(m_monkeyController.x()).whileTrue( new SetElevatorCommand(elevator, MechanismConstants.ELEVATOR_INTAKE_HIGHT)).onFalse(new CoralElevatorCommand(elevator, 2.5)); // coral intake make it go up then down.
    m_monkeyController.leftBumper().negate().and(m_monkeyController.a()).whileTrue(new ParallelCommandGroup(new SetElevatorCommand(elevator, MechanismConstants.ELEVATOR_CORAL_L2_HIGHT), new DeployAlgaeCommand(endEffector, -0.25, false))).onFalse(DeployCoral); //L2 1.2
    m_monkeyController.leftBumper().negate().and(m_monkeyController.b()).whileTrue(new ParallelCommandGroup(new SetElevatorCommand(elevator, MechanismConstants.ELEVATOR_CORAL_L3_HIGHT), new DeployAlgaeCommand(endEffector, -0.25, false))).onFalse(DeployCoral); //L3
    m_monkeyController.leftBumper().negate().and(m_monkeyController.y()).whileTrue(new ParallelCommandGroup(new SetElevatorCommand(elevator, MechanismConstants.ELEVATOR_CORAL_L4_HIGHT), new DeployAlgaeCommand(endEffector, -0.25, false))).onFalse(new SequentialCommandGroup(new DeployCoralL4Command(elevator, endEffector), new LowerElevatorCommand(elevator, endEffector))); //L4
    
    m_monkeyController.rightTrigger().whileTrue(new ParallelCommandGroup(new DeployAlgaeCommand(endEffector, 0.5, false), new MidIntakeAlgae(intake)));

    m_monkeyController.leftTrigger().whileTrue(new DeployAlgaeCommand(endEffector, -0.5, false));

    m_monkeyController.start().onTrue(new RetractntakeArmsCommand(intake));

  }

  // Use this to pass the autonomous command to the main {@link Robot} class.
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
