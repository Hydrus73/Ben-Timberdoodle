// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class ElevatorConstants {
        
    public static final int leftID = 0;
    public static final int rightID = 0;

    public static final double GEARBOX_RATIO = 11.25;

    public static final double SPOOL_RADIUS = Units.inchesToMeters(1.25);
    public static final double SPOOL_CIRCUMFERENCE = 2*Math.PI*SPOOL_RADIUS;

    public static final double MAX_HEIGHT = 1.65;

    public static final double POSITION_CONVERSION_FACTOR = SPOOL_CIRCUMFERENCE/GEARBOX_RATIO;
    public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR/60.0;

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;

  }

  public static final class IntakeConstant {

    public static final int intakeID = 0;

  }

  public static final class DriveConstants {
    public static final int frontLeftTurnID = 3;
    public static final int frontLeftDriveID = 4;
    public static final int frontLeftCancoderID = 0;
    public static final double frontLeftOffset = 0.556+0.25;

    public static final int frontRightTurnID = 5;
    public static final int frontRightDriveID = 6;
    public static final int frontRightCancoderID = 1;
    public static final double frontRightOffset = 0.655-0.25;

    public static final int backLeftTurnID = 7;
    public static final int backLeftDriveID = 8;
    public static final int backLeftCancoderID = 2;
    public static final double backLeftOffset = 0.263+0.25;

    public static final int backRightTurnID = 1;
    public static final int backRightDriveID = 2;
    public static final int backRightCancoderID = 3;
    public static final double backRightOffset = 0.285+0.25;

    //m/s
    public static final double MAX_LINEAR_VELOCITY = Units.feetToMeters(16);
    //rad/s
    public static final double MAX_TURN_VELOCITY = 11;
}

public static class ModuleConstants {

    public static final double STEER_RATIO = 150.0/7.0;
    public static final double DRIVE_RATIO = 6.12;

    public static final double WHEEL_CIRCUMPHRENCE = Units.inchesToMeters(4 * Math.PI);

    public static final double kModuleToModuleDistance = Units.inchesToMeters(19.750);
    public static final double kModuleToCenter = kModuleToModuleDistance / 2;

    public static final double driveP = 0.01;
    public static final double driveI = 0.00;
    public static final double driveD = 0.0;
    public static final double driveFF = 1.96;

    public static final double turnP = .01;
    public static final double turnI = 0.0;
    public static final double turnD = 0.005;

}

public static final class VisionConstants {
    // Meters
    public static final double cameraHeight = Units.inchesToMeters(4.5);//1;

    // Camera angle offset
    public static final double cameraOffset = 0.0;

    // POSE Estimations
    public static final int POSE_ESTIMATIONS = 30;

    // MAX POSE time kept millis
    public static final double TIME_KEPT = 1500;
}

public static final class ArmConstants {

    public static final int armId = 13;

    public static final double GEARBOX_RATIO = 25;

    /**
     * Meters
     */
    public static final double ARM_LENGTH = 1;

    // Arm MAX angle limit
    public static final double UPPER_LIMIT = Units.degreesToRadians(45);
    // Arm Starting angle
    public static final double STARTING_ANGLE = 0.0;
    // Arm MIN angle limit
    public static final double LOWER_LIMIT = Units.degreesToRadians(-45);

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double kFF = 0.0;

}

public static class GyroConstants {
    public static final int GYRO_ID = 2;

}

public static class WristConstants {

  public static final int LEFT_ID = 0;
  public static final int RIGHT_ID = 0;

  public static final double GEAR_RATIO = 65.64;

  public static final double POSITION_CONVERSION_FACTOR = 2*Math.PI/GEAR_RATIO;
  public static final double VELOCITY_CONVERSION_FACTOR = POSITION_CONVERSION_FACTOR/60.0;

  public static final double kP = 0.0;
  public static final double kI = 0.0;
  public static final double kD = 0.0;
  public static final double kFF = 0.0;
}

public static class LightsConstants {
  public static final int PORT = 0;
  public static final int COUNT = 1000;
}

}
