package frc.robot.Drivetrain.Commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.DriveConstants;
import frc.robot.Drivetrain.DrivetrainSubsystem;

public class AssistedDrive extends CommandBase {
    

    private final DoubleSupplier x_trans;
    private final DoubleSupplier y_trans;
    private final DoubleSupplier rot;

    private final DrivetrainSubsystem drive;

    private final Rotation2d offset;

    public AssistedDrive(DrivetrainSubsystem d, DoubleSupplier x, DoubleSupplier y, DoubleSupplier r) {
        x_trans = x;
        y_trans = y;
        rot = r;
        drive = d;
        offset = new Rotation2d(DriverStation.getAlliance() == Alliance.Red ? Math.PI : 0.0);
        addRequirements(drive);
    }

    @Override
    public void initialize() {}

    @Override
    public void execute() {
        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            y_trans.getAsDouble()*-DriveConstants.MAX_LINEAR_VELOCITY, 
            x_trans.getAsDouble()*DriveConstants.MAX_LINEAR_VELOCITY, 
            rot.getAsDouble()*DriveConstants.MAX_TURN_VELOCITY,
            drive.getRobotAngle().plus(offset));
        
        // drive.drive(new ChassisSpeeds(
        //     y_trans.getAsDouble()*DriveConstants.MAX_LINEAR_VELOCITY, 
        //     x_trans.getAsDouble()*DriveConstants.MAX_LINEAR_VELOCITY, 
        //     rot.getAsDouble()*DriveConstants.MAX_TURN_VELOCITY
        // ));
    }

    @Override
    public void end(boolean interr) {
        drive.drive(
            new ChassisSpeeds(0.0, 0.0, 0.0)
        );
    }

}