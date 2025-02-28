// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorBargeCommand extends Command {
  private IntakeSubsystem intake;
  private Timer timer;
  private ElevatorSubsystem elevator;
  /** Lowers intake arms and move the elevetor to the Barge Poisition */
  public ElevatorBargeCommand(IntakeSubsystem intake, ElevatorSubsystem elevator) {
    this.elevator = elevator;
    timer = new Timer();
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
     intake.SetAngle(MechanismConstants.INTAKE_ARM_MID);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(timer.get() > 1.0){
      elevator.setRotations(MechanismConstants.ELEVATORR_BARGE);
    }
  }

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
