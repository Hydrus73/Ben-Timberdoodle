package frc.robot.Sensors.Field;

import java.util.Map;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;

public class FieldElements {

    public static final AprilTag[] tags = new AprilTag[] {
        new AprilTag(1,
                new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22),
                        new Rotation3d(0, 0, Math.PI))),
        new AprilTag(2,
                new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22),
                        new Rotation3d(0, 0, Math.PI))),
        new AprilTag(3,
                new Pose3d(Units.inchesToMeters(610.77), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22),
                        new Rotation3d(0, 0, Math.PI))),
        new AprilTag(4,
                new Pose3d(Units.inchesToMeters(636.96), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38),
                        new Rotation3d(0, 0, Math.PI))),
        new AprilTag(5,
                new Pose3d(Units.inchesToMeters(14.25), Units.inchesToMeters(265.74), Units.inchesToMeters(27.38),
                        new Rotation3d(0, 0, 0))),
        new AprilTag(6,
                new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(174.19), Units.inchesToMeters(18.22),
                        new Rotation3d(0, 0, 0))),
        new AprilTag(7,
                new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(108.19), Units.inchesToMeters(18.22),
                        new Rotation3d(0, 0, 0))),
        new AprilTag(8,
                new Pose3d(Units.inchesToMeters(40.45), Units.inchesToMeters(42.19), Units.inchesToMeters(18.22),
                        new Rotation3d(0, 0, 0))),
};

    public static final Map<Integer, AprilTag> aprilTags = Map.ofEntries(
        Map.entry(1, tags[0]),
        Map.entry(2, tags[1]),
        Map.entry(3, tags[2]),
        Map.entry(4, tags[3]),
        Map.entry(5, tags[4]),
        Map.entry(6, tags[5]),
        Map.entry(7, tags[6]),
        Map.entry(8, tags[7])
    );

    public static final Map<Integer, Pose2d> scoringStations = Map.ofEntries(
        Map.entry(1, new Pose2d(Units.inchesToMeters(596.6), Units.inchesToMeters(20.5), new Rotation2d())),
        Map.entry(2, new Pose2d(Units.inchesToMeters(596.6), Units.inchesToMeters(42.4), new Rotation2d())),
        Map.entry(3, new Pose2d(Units.inchesToMeters(596.6), Units.inchesToMeters(64.4), new Rotation2d())),
        Map.entry(4, new Pose2d(Units.inchesToMeters(596.6), Units.inchesToMeters(86.4), new Rotation2d())),
        Map.entry(5, new Pose2d(Units.inchesToMeters(596.6), Units.inchesToMeters(108.4), new Rotation2d())),
        Map.entry(6, new Pose2d(Units.inchesToMeters(596.6), Units.inchesToMeters(130.4), new Rotation2d())),
        Map.entry(7, new Pose2d(Units.inchesToMeters(596.6), Units.inchesToMeters(152.4), new Rotation2d())),
        Map.entry(8, new Pose2d(Units.inchesToMeters(596.6), Units.inchesToMeters(174.4), new Rotation2d())),
        Map.entry(9, new Pose2d(Units.inchesToMeters(596.6), Units.inchesToMeters(196.4), new Rotation2d())),
        Map.entry(10, new Pose2d(Units.inchesToMeters(53.8), Units.inchesToMeters(196.4), new Rotation2d())),
        Map.entry(11, new Pose2d(Units.inchesToMeters(53.8), Units.inchesToMeters(174.4), new Rotation2d())),
        Map.entry(12, new Pose2d(Units.inchesToMeters(53.8), Units.inchesToMeters(152.4), new Rotation2d())),
        Map.entry(13, new Pose2d(Units.inchesToMeters(53.8), Units.inchesToMeters(130.4), new Rotation2d())),
        Map.entry(14, new Pose2d(Units.inchesToMeters(53.8), Units.inchesToMeters(108.4), new Rotation2d())),
        Map.entry(15, new Pose2d(Units.inchesToMeters(53.8), Units.inchesToMeters(86.4), new Rotation2d())),
        Map.entry(16, new Pose2d(Units.inchesToMeters(53.8), Units.inchesToMeters(64.4), new Rotation2d())),
        Map.entry(17, new Pose2d(Units.inchesToMeters(53.8), Units.inchesToMeters(42.4), new Rotation2d())),
        Map.entry(18, new Pose2d(Units.inchesToMeters(53.8), Units.inchesToMeters(20.5), new Rotation2d()))
    );
    
}
