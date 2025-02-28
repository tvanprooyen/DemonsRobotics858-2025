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
public class LowerElevatorAlgaeCommand extends Command {
  
  ElevatorSubsystem elevator;
  IntakeSubsystem intake;
  Timer timer = new Timer();

  /** Lowers intake arms to make space for the algae to lower with the elevator */
  public LowerElevatorAlgaeCommand(ElevatorSubsystem elevator, IntakeSubsystem intake) {
    this.elevator = elevator;
    this.intake = intake;
    addRequirements(elevator, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.SetAngle(MechanismConstants.INTAKE_ARM_MID);
    elevator.setRotations(0);
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      if(elevator.EncoderRotations() < 0.1 || timer.get() > 2.5){
        intake.SetAngle(MechanismConstants.INTAKE_ARM_UP);
      }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 3.0; 
  }
}
