package frc.robot.Auto;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Drivetrain.DrivetrainSubsystem;
import frc.robot.Sensors.Gyro.Gyro;

public class AutoLevel extends CommandBase {

    private final DrivetrainSubsystem d;

    private final PigeonIMU gyro;

    public AutoLevel(DrivetrainSubsystem d) {
        this.d = d;
        this.gyro = Gyro.getInstance().getGyro();
        addRequirements(d);
    }

    @Override
    public void execute() {
        Rotation2d yaw = Gyro.getInstance().getAngle();
        Rotation2d pitch = Rotation2d.fromDegrees(Gyro.getInstance().getGyro().getPitch());
        Rotation2d roll = Rotation2d.fromDegrees(Gyro.getInstance().getGyro().getRoll());

        // Radians
        double rollZ = (Math.pow(yaw.getCos(), 2)*pitch.getRadians() + Math.pow(yaw.getSin(), 2)*roll.getRadians());
        double pitchZ = Math.pow(yaw.getCos(), 2)*roll.getRadians() + Math.pow(yaw.getSin(), 2)*pitch.getRadians();

        SmartDashboard.putNumber("Yaw", yaw.getRadians());
        SmartDashboard.putNumber("Pitch Z", pitchZ);
        SmartDashboard.putNumber("Roll Z", rollZ);

        if (Math.abs(pitchZ) < 0.1) pitchZ = 0.0;


        d.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(pitchZ, 0.0, 0.0, d.getRobotAngle())
        );

    }
    
}