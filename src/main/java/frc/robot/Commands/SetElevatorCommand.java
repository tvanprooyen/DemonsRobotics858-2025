// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;


import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class SetElevatorCommand extends Command {

  private ElevatorSubsystem elevator;
  private double rotations;

  /** Moves the elevator to a set height. */
  public SetElevatorCommand(ElevatorSubsystem elevator, double rotations) {
    this.rotations = rotations;
    this.elevator = elevator;
    addRequirements(this.elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setRotations(rotations);
  }

  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
