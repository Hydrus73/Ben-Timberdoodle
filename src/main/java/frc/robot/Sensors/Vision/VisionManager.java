package frc.robot.Sensors.Vision;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sensors.Field.FieldElements;
import frc.robot.Sensors.Field.PositionManager;
import frc.robot.Sensors.Gyro.Gyro;
import frc.robot.Sensors.Vision.Vision.StereoTarget;

public class VisionManager extends SubsystemBase {

    private static VisionManager instance;
    
    private Vision vision = new Vision();

    private final ArrayList<Map.Entry<Double, Pose2d>> positions = new ArrayList<>();

    private VisionManager() {
        for (int i = 0; i < FieldElements.tags.length; i++) {
            PositionManager.getInstance().getObject(String.valueOf(i)).setPose(FieldElements.tags[i].pose.toPose2d());
        }
    }

    public static VisionManager getInstance() {
        if (instance == null) {
            instance = new VisionManager();
        }
        return instance;
    }

    @Override
    public void periodic() {
        positions.clear();
        // for (Target t : vision.getTargets()) {
        //     continue;
        //     // positions.add(new Pose2d());
        // }

        for (StereoTarget t : vision.getStereoTargets()) {
            positions.add(Map.entry(1.0/(double)vision.getStereoTargets().size(), vision.estimateStereoTargetPos(t, Gyro.getInstance().getAngle())));
        }
    }

    public Pose2d getEstimatedPose() {
        double x = 0.0;
        double y = 0.0;
        for (Map.Entry<Double, Pose2d> e : positions) {
            x += e.getKey()*e.getValue().getX();
            y += e.getKey()*e.getValue().getY();
        }
        return new Pose2d(x, y, Gyro.getInstance().getAngle());
    }

}