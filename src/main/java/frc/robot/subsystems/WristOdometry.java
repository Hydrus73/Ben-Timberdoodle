// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class WristOdometry {
    double position, velocity;
    public WristOdometry(double position, double velocity) {
        this.position = position;
        this.velocity = velocity;
    }
    public void update(double position, double velocity) {
        this.position = position;
        this.velocity = velocity;
        SmartDashboard.putNumber("Wrist Position", position);
        SmartDashboard.putNumber("Wrist Velocity", velocity);
    }
    public String toString() {
        String output = "Wrist Odometry(Position: "+position+", Velocity: "+velocity+")";
        return output;
    }
}
