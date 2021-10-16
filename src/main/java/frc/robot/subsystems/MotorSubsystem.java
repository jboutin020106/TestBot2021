// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class MotorSubsystem extends SubsystemBase {
  private TalonSRX leftSide;
  private TalonSRX rightSide;
  private TalonSRX hDriveMotor;
  private boolean toggle = false;

  /** Creates a new MotorSubsystem. */
  public MotorSubsystem() {
    rightSide = new TalonSRX(Constants.rightMotor);
    leftSide = new TalonSRX(Constants.leftMotor);
    hDriveMotor = new TalonSRX(Constants.HDriveMotor);
    
    rightSide.setInverted(true);
  
  

  }


 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void spinMotor(double rightSpeed, double leftSpeed){
    if(!toggle){
    leftSide.set(ControlMode.PercentOutput, leftSpeed);
    rightSide.set(ControlMode.PercentOutput, rightSpeed);
    }
    else if(toggle){
    hDriveMotor.set(ControlMode.PercentOutput, rightSpeed);
    }
  }
  public void changeToggle(){
    toggle = !toggle;
    System.out.println(toggle);
  }

}
