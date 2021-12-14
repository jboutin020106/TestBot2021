// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.MotorSubsystem;

public class SpinCommand extends CommandBase {
  private MotorSubsystem spinningFunction;
  private Joystick joy;
 

  

  
    // Use addRequirements() here to declare subsystem dependencies.
  public SpinCommand(MotorSubsystem motorSubsys, Joystick joy) {
      this.spinningFunction =  motorSubsys;
      this.joy = joy;
      addRequirements(spinningFunction); 
    
  }
 
  


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    spinningFunction.spinMotor(joy.getY() * .6, joy.getRawAxis(4) * .6);
    System.out.println("Right Joy: " + joy.getY());
    System.out.println("Left Joy: " + joy.getRawAxis(4));
    SmartDashboard.putNumber("Right motor teleop speed", joy.getY());
    SmartDashboard.putNumber("Left motor teleop speed", joy.getRawAxis(4));


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    
    spinningFunction.spinMotor(0,0);
    SmartDashboard.putNumber("Right motor teleop speed", joy.getY());
    SmartDashboard.putNumber("Left motor teleop speed", joy.getRawAxis(4));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
