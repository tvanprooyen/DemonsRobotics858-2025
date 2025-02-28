// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeployCoralL4Command extends Command {
  
  private ElevatorSubsystem elevatorSubsystem;
  private EndEffectorSubsystem endEffectorSubsystem;
  private Timer timer;

  /** Move the elevator up and ejects coral at the same time for L4 */
  public DeployCoralL4Command(ElevatorSubsystem elevatorSubsystem, EndEffectorSubsystem endEffectorSubsystem) {
    timer = new Timer();
    this.elevatorSubsystem = elevatorSubsystem;
    this.endEffectorSubsystem = endEffectorSubsystem;
     addRequirements(endEffectorSubsystem, elevatorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    elevatorSubsystem.setRotations(7.5); //Changed from 6.5 to 7 2/15/2025
    endEffectorSubsystem.RunTopWheel(0.4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
   }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 0.6;
  }
}
