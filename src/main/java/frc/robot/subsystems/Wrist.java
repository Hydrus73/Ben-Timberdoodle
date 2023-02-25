// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Auto.SystemsCheck.SubChecker;
import frc.robot.Auto.SystemsCheck.SystemsCheck;

public class Wrist extends SubsystemBase implements SubChecker {
  WristOdometry wristOdometry;
  CANSparkMax left, right;
  SparkMaxPIDController leftPID;
  RelativeEncoder leftEncoder;
  AbsoluteEncoder absoluteEncoder;
  /** Creates a new Wrist. */
  public Wrist() {
    left = new CANSparkMax(Constants.WristConstants.LEFT_ID, MotorType.kBrushless);
    right = new CANSparkMax(Constants.WristConstants.RIGHT_ID, MotorType.kBrushless);
    left.setInverted(true);
    right.setInverted(false);
    right.follow(left);
    leftPID = left.getPIDController();
    leftPID.setP(Constants.WristConstants.kP);
    leftPID.setI(Constants.WristConstants.kI);
    leftPID.setD(Constants.WristConstants.kD);
    leftPID.setFF(Constants.WristConstants.kFF);
    leftEncoder = left.getEncoder();
    absoluteEncoder = left.getAbsoluteEncoder(Type.kDutyCycle);
    double startPosition = absoluteEncoder.getPosition()*2*Math.PI;
    leftEncoder.setPosition(startPosition);
    leftEncoder.setPositionConversionFactor(Constants.WristConstants.POSITION_CONVERSION_FACTOR);
    leftEncoder.setVelocityConversionFactor(Constants.WristConstants.VELOCITY_CONVERSION_FACTOR);
    wristOdometry = new WristOdometry(startPosition, 0);
  }

  public void setPosition(double angle) {
    leftPID.setReference(angle, ControlType.kPosition);
  }

  public void reset() {
    setPosition(0);
  }

  public void setVelocity(double angularSpeed) {
    leftPID.setReference(angularSpeed, ControlType.kVelocity);
  }

  @Override
  public void periodic() {
    wristOdometry.update(leftEncoder.getPosition(), leftEncoder.getVelocity());
  }

  public Command check(boolean safe) {
    if (!safe) {
      return new InstantCommand();
    }
    return new InstantCommand(() -> {
      double time = System.currentTimeMillis();
      boolean working = false;
      setPosition(Math.PI/6.0);
      while (System.currentTimeMillis() - time < 2000) {}
      if (leftEncoder.getPosition() > Math.PI/12.0) {
        working = true;
      }
      SystemsCheck.setSystemStatus(this, working);
      reset();
    }, this);
  }

}
