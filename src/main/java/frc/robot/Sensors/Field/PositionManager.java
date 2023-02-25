package frc.robot.Sensors.Field;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;

public class PositionManager {

    private static PositionManager instance;

    private Field2d field;
    private Pose2d position;

    public static PositionManager getInstance() {
        if (instance == null) {
            instance = new PositionManager();
        }
        return instance;
    }

    private PositionManager() {
        field = new Field2d();
        position = new Pose2d();
        Shuffleboard.getTab("Field").add("Field", field);
    }
    
    public Field2d getField() {
        return field;
    }

    public Rotation2d getRobotRotation() {
        return position.getRotation();
    }

    public Pose2d getRobotPose() {
        return position;
    }

    public void setRobotPose(Pose2d p) {
        position = p;
        field.getRobotObject().setPose(p);
    }

    public FieldObject2d getObject(String n) {
        return field.getObject(n);
    }

    public Pose2d poseOfClosestScoring() {
        Pose2d closest = new Pose2d();
        Pose2d currentPos = getRobotPose();
        for (Pose2d p : FieldElements.scoringStations.values()) {
            if (currentPos.getTranslation().getDistance(p.getTranslation()) < currentPos.getTranslation().getDistance(closest.getTranslation())) {
                closest = p;
            }
        }
        return closest;
    }

}