// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.EndEffectorSubsystem;

public class DeployAlgaeCommand extends Command {
  private EndEffectorSubsystem endEffector;
  private double speed;
  private boolean onTilt;
  private Timer timer;

  /** Ejects or Intakes algae. */
  public DeployAlgaeCommand(EndEffectorSubsystem endEffector, double speed, boolean onTilt) {
    timer = new Timer();
    this.endEffector = endEffector;
    this.speed = speed;
    this.onTilt = onTilt;
    addRequirements(endEffector);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(onTilt){
      endEffector.TiltEndeffector(onTilt);
      timer.reset();
      timer.start();

    } else{
      endEffector.setBothWheels(speed);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(onTilt){
      if(timer.get() > 0.6){
        endEffector.setBothWheels(speed);
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    endEffector.setBothWheels(0);
    endEffector.TiltEndeffector(false);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
