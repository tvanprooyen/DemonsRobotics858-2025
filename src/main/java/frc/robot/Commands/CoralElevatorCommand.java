// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralElevatorCommand extends Command {
  private ElevatorSubsystem elevator;
  private double rotations;
  private Timer timer;

  /** When triggerd it moves the elevator to the set height and then goes down. */
  public CoralElevatorCommand(ElevatorSubsystem elevator, double rotations) {
    this.elevator =  elevator;
    timer = new Timer();
    this.rotations = rotations;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    elevator.setRotations(rotations);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() > 0.5){ // Original Value 1 second
      elevator.setRotations(0.0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 1.5;
  }
}
