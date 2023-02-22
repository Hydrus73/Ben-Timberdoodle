// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elevator extends SubsystemBase {
  ElevatorOdometry elevatorOdometry;
  CANSparkMax left, right;
  SparkMaxPIDController leftPID;
  RelativeEncoder leftEncoder;
  /** Creates a new Elevator. */
  public Elevator() {
    elevatorOdometry = new ElevatorOdometry(0, 0);
    left = new CANSparkMax(Constants.ElevatorConstants.leftID, MotorType.kBrushless);
    right = new CANSparkMax(Constants.ElevatorConstants.rightID, MotorType.kBrushless);
    left.setInverted(true);
    right.setInverted(false);
    right.follow(left);
    leftPID = left.getPIDController();
    leftPID.setP(Constants.ElevatorConstants.kP);
    leftPID.setI(Constants.ElevatorConstants.kI);
    leftPID.setD(Constants.ElevatorConstants.kD);
    leftPID.setFF(Constants.ElevatorConstants.kFF); 
    leftEncoder = left.getEncoder();
    leftEncoder.setPositionConversionFactor(Constants.ElevatorConstants.POSITION_CONVERSION_FACTOR);
    leftEncoder.setVelocityConversionFactor(Constants.ElevatorConstants.VELOCITY_CONVERSION_FACTOR);
  }

  public void setPosition(double position) {
    leftPID.setReference(position, ControlType.kPosition);
  }

  public void setVelocity(double velocity) {
    leftPID.setReference(velocity, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    elevatorOdometry.update(leftEncoder.getPosition(), leftEncoder.getVelocity());
  }
}
