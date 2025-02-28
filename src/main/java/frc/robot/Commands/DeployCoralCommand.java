// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class DeployCoralCommand extends Command {
  
  private EndEffectorSubsystem endEffector;
  private double speed;
  private Timer timer;
  /** Ejects coral at a certain speed. */
  public DeployCoralCommand(EndEffectorSubsystem endEffector, double speed) {
    timer = new Timer();
    this.speed = speed;
    this.endEffector = endEffector;
    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {


    if(timer.get() > 0.5){
      endEffector.RunTopWheel(0);
    } else {
      endEffector.RunTopWheel(speed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    endEffector.RunTopWheel(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 1.5;
  }
}
