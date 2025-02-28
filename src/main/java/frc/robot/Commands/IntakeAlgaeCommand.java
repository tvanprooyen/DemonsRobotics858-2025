// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MechanismConstants;
import frc.robot.subsystems.EndEffectorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAlgaeCommand extends Command {
  private IntakeSubsystem intake;
  private EndEffectorSubsystem endEffector;
  private double intakeSpeed;
  private double endEffectorSpeed;

  /* Lowers intake arms and intakes algae from the ground */
  public IntakeAlgaeCommand(IntakeSubsystem intake, EndEffectorSubsystem endEffector, double intakeSpeed, double endEffectorSpeed) {
      this.intake = intake;
      this.endEffector = endEffector;
      this.intakeSpeed = intakeSpeed;
      this.endEffectorSpeed = endEffectorSpeed;
      addRequirements(intake, endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.SetFeedMotors(intakeSpeed);
    intake.SetAngle(MechanismConstants.INTAKE_ARM_DOWN);
    endEffector.setBothWheels(endEffectorSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.SetFeedMotors(0.0);
    endEffector.setBothWheels(0);
    intake.SetAngle(MechanismConstants.INTAKE_ARM_UP);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
