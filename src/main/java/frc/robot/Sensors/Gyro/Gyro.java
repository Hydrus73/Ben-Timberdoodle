package frc.robot.Sensors.Gyro;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.GyroConstants;

public class Gyro {

    private static Gyro instance;

    private PigeonIMU pigeon;

    public static Gyro getInstance() {
        if (instance == null) {
            instance = new Gyro(GyroConstants.GYRO_ID);
        }
        return instance;
    }

    public Gyro(int id) {
        pigeon = new PigeonIMU(id);
        pigeon.configFactoryDefault();
        pigeon.setYaw(DriverStation.getAlliance() == Alliance.Blue ? 180 : 0);
    }

    /**
     * Gets the pigeon angle
     * [-180 deg, 180 deg] 0 IS POINTING AWAY FROM BLUE ALLIANCE
     * @return Rotation2d of the gyro angle
     */
    public Rotation2d getAngle() {
        return Rotation2d.fromDegrees(((pigeon.getYaw()%360)+360)%360-180);
    }

    public void setAngle(double a) {
        pigeon.setYaw(a);
    }

    public PigeonIMU getGyro() {
        return pigeon;
    }
    
}
