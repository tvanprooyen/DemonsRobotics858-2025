// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.EndEffectorSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LowerElevatorCommand extends Command {
   private ElevatorSubsystem elevator;
   private EndEffectorSubsystem endEffector;

  /** It lowers the elevetor to zero and set the endeffector wheels to zero */
  public LowerElevatorCommand(ElevatorSubsystem elevator, EndEffectorSubsystem endEffector) {
    this.endEffector = endEffector;
    this.elevator = elevator;
    addRequirements(elevator, endEffector);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setRotations(0.0);
    endEffector.setBothWheels(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.EncoderRotations() < 0.1;
  }
}
