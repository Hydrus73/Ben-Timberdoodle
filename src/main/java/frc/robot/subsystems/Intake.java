// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Auto.SystemsCheck.SubChecker;
import frc.robot.Auto.SystemsCheck.SystemsCheck;

import frc.robot.Constants;

public class Intake extends SubsystemBase implements SubChecker {
  CANSparkMax intake;
  RelativeEncoder intakeEncoder;
  /** Creates a new Intake. */
  public Intake() {
    intake = new CANSparkMax(Constants.IntakeConstant.intakeID, MotorType.kBrushless);
    intakeEncoder = intake.getEncoder();
  }

  public void in() {
    intake.set(0.5);
  }

  public void out() {
    intake.set(0.5);
  }

  public void stop() {
    intake.set(0);
  }

  public void intake(double speed) {
    intake.set(speed);
  }

  public Command check(boolean safe) {
    if (!safe) {
      return new InstantCommand();
    }
    return new InstantCommand(() -> {
      double time = System.currentTimeMillis();
      boolean working = false;
      intake(0.3);
      while (System.currentTimeMillis() - time < 1000) {}
      if (intakeEncoder.getVelocity() > 0.1) {
        working = true;
      }
      SystemsCheck.setSystemStatus(this, working);
      stop();
    }, this);
  }

}
