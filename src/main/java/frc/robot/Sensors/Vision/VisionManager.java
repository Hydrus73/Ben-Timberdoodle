package frc.robot.Sensors.Vision;

import java.util.ArrayList;
import java.util.Map;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Sensors.Field.FieldElements;
import frc.robot.Sensors.Field.PositionManager;
import frc.robot.Sensors.Gyro.Gyro;
import frc.robot.Sensors.Vision.Vision.StereoTarget;
import frc.robot.Sensors.Vision.Vision.Target;

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

    public Rotation2d getEstimatedGyro() {
        ArrayList<StereoTarget> targets = vision.getStereoTargets();
        if (targets.size() < 2) {
            return null; // both can't see the same one
        }
        else {
            StereoTarget first = targets.get(0);
            StereoTarget second = targets.get(0);
            boolean match = false;
            loop:
            for (int i = 0; i < targets.size() - 1; i++) {
                first = targets.get(i);
                for (int x = i + 1; x < targets.size(); x++) {
                    second = targets.get(x);
                    if (first.leftTarget.tagId == second.leftTarget.tagId && first.rightTarget.tagId == second.rightTarget.tagId) {
                        match = true;
                        break loop;
                    }
                }
            }
            if (!match) {
                return null;
            }
            Target leftTarget = first.leftTarget;
            Target rightTarget = first.rightTarget;
            ArrayList<Target> leftTargets = first.leftCam.getTargets();
            int leftLeftIndex=0, leftRightIndex=0;
            for (int i = 0; i < leftTargets.size(); i++) {
                if (leftTargets.get(i).tagId == leftTarget.tagId) {
                    leftLeftIndex = i;
                }
                if (leftTargets.get(i).tagId == rightTarget.tagId) {
                    leftRightIndex = i;
                }
            }
            ArrayList<Target> rightTargets = first.rightCam.getTargets();
            int rightLeftIndex=0, rightRightIndex=0;
            for (int i = 0; i < rightTargets.size(); i++) {
                if (rightTargets.get(i).tagId == leftTarget.tagId) {
                    rightLeftIndex = i;
                }
                if (rightTargets.get(i).tagId == rightTarget.tagId) {
                    rightRightIndex = i;
                }
            }
            double theta1 = leftTargets.get(leftLeftIndex).xOffset;
            double theta2 = rightTargets.get(rightLeftIndex).xOffset;
            double alpha1 = leftTargets.get(leftRightIndex).xOffset;
            double alpha2 = rightTargets.get(rightRightIndex).xOffset;
            theta1 = Units.degreesToRadians(theta1);
            theta2 = Units.degreesToRadians(theta2);
            alpha1 = Units.degreesToRadians(alpha1);
            alpha2 = Units.degreesToRadians(alpha2);
            double t2y = Units.inchesToMeters(66);
            double[] c2 = new double[2];
            c2[0] = t2y*Math.cos(alpha2)*Math.cos(theta2)/Math.sin(alpha2-theta2);
            c2[1] = c2[0]*Math.tan(alpha2);
            double[] c1 = new double[2];
            c1[0] = Math.tan(alpha1)/Math.tan(theta1);
            c1[1] = c1[0]*Math.tan(theta1);
            double angle = Math.atan((c1[1]-c2[1])/(c1[0]-c2[0]));
            return new Rotation2d(angle);
        }
    }

}