// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import frc.robot.Auto.SystemsCheck.SubChecker;
import frc.robot.Auto.SystemsCheck.SystemsCheck;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

public class Elevator extends SubsystemBase implements SubChecker {
  ElevatorOdometry elevatorOdometry;
  CANSparkMax left, right;
  SparkMaxPIDController leftPID;
  RelativeEncoder leftEncoder;
  SparkMaxAbsoluteEncoder absoluteEncoder;
  /** Creates a new Elevator. */
  public Elevator() {
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
    absoluteEncoder = left.getAbsoluteEncoder(Type.kDutyCycle);
    double startRotation = absoluteEncoder.getPosition();
    double startPosition = startRotation*Constants.ElevatorConstants.SPOOL_CIRCUMFERENCE;
    leftEncoder.setPosition(startPosition);
    elevatorOdometry = new ElevatorOdometry(startPosition, 0);
  }

  public void setPosition(double position) {
    leftPID.setReference(position, ControlType.kPosition);
  }

  public void reset() {
    setPosition(0);
  }

  public void setVelocity(double velocity) {
    leftPID.setReference(velocity, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    elevatorOdometry.update(leftEncoder.getPosition(), leftEncoder.getVelocity());
  }

  public Command check(boolean safe) {
    if (!safe) {
      return new InstantCommand();
    }
    return new InstantCommand(() -> {
      double time = System.currentTimeMillis();
      boolean working = false;
      setPosition(0.1);
      while (System.currentTimeMillis() - time < 2000) {}
      if (leftEncoder.getPosition() > 0.05) {
        working = true;
      }
      SystemsCheck.setSystemStatus(this, working);
      reset();
    }, this);
  }

}
