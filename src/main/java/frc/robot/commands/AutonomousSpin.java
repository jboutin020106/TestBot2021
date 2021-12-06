// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MotorSubsystem;

public class AutonomousSpin extends CommandBase {
   private MotorSubsystem autoSpinning;
   private double motorSpeedLeft;
   private double motorSpeedRight;

  /** Creates a new AutonomousSpin. */
  public AutonomousSpin(MotorSubsystem autoSubsystem, double speedLeft, double speedRight) {
    this.autoSpinning = autoSubsystem;
    motorSpeedLeft = speedLeft;
    motorSpeedRight = speedRight;
    
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(autoSpinning);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    autoSpinning.spinMotor(motorSpeedLeft, motorSpeedRight);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    autoSpinning.spinMotor(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
