// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.MotorSubsystem;


public class RotateToAngleCommand extends CommandBase {
  /** Creates a new RotateToAngleCommand. */

  private MotorSubsystem rotateSubsystem;
  private double angle;
  private double gyro_angle;
  
  
  private boolean isTurningRight;

  private static final double normalRotateSpeed = 0.6;
  private static final double slowRotateSpeed = 0.3;
  private static final double margin = 10;
  private static final double slowMargin = 120;

 

  



  public RotateToAngleCommand(int angle, MotorSubsystem rotateMotorSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    rotateSubsystem = rotateMotorSubsystem;
    this.angle = angle;
    addRequirements(rotateSubsystem);
  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rotateSubsystem.zeroNavX();

    gyro_angle = rotateSubsystem.getNavXAngle();

      if(Math.abs(angle) > 180){
        
        isTurningRight = false;

      }
      else{

        isTurningRight = true;

      }


    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      double currentDrivingSpeed;
      gyro_angle = rotateSubsystem.getNavXAngle();
      boolean needsSlowMode = (gyro_angle) <= (angle) + slowMargin && (gyro_angle) >= angle - slowMargin;
     

      if(needsSlowMode){
        currentDrivingSpeed = slowRotateSpeed;
      }
      else{
        currentDrivingSpeed = normalRotateSpeed;
      }

      if(isTurningRight){
        rotateSubsystem.spinMotor(currentDrivingSpeed, currentDrivingSpeed * -1.0);
      }
      else{
        rotateSubsystem.spinMotor(currentDrivingSpeed * -1.0, currentDrivingSpeed);
      }
  }

  public boolean isFinished() {
    boolean isFinished = (gyro_angle) <= angle + margin && (gyro_angle) >= (angle) - margin;
    if(isFinished){
        return true;
    }
    else{
        return false;
    }

  }

  protected void end(){
    rotateSubsystem.spinMotor(0.0, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    end();
  }

  // Returns true when the command should end.
  
  

        
    
  }

