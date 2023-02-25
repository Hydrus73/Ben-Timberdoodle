package frc.robot.Sensors.Vision;

import java.util.ArrayList;

import org.photonvision.PhotonCamera;
import org.photonvision.SimVisionSystem;
import org.photonvision.SimVisionTarget;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.Robot;
import frc.robot.Sensors.Field.PositionManager;
import frc.robot.Sensors.Vision.Vision.Target;

public class Camera extends PhotonCamera {

    private final Transform3d trans;
    private Camera pair;

    private final ArrayList<Target> targets = new ArrayList<>();

    private final SimVisionSystem simCam;

    /**
     * @param name Photon camera name
     * @param trans Camera position relative to the center
     */
    public Camera(String name, Transform3d trans) {
        super(name);
        this.trans = trans;
        this.pair = null;
        simCam = new SimVisionSystem(
            name, 
            70,
            new Transform3d(new Translation3d(this.trans.getX(), this.trans.getY(), this.trans.getZ()), new Rotation3d()),
            10,
            1280,
            720,
            10);
    }

    /**
     * @param name Photon camera name
     * @param trans Camera position relative to the center
     * @param pair Stereo camera pair object
     */
    public Camera(String name, Transform3d trans, Camera pair) {
        super(name);
        this.trans = trans;
        this.pair = pair;

        simCam = new SimVisionSystem(
            name, 
            70,
            new Transform3d(new Translation3d(this.trans.getX(), this.trans.getY(), this.trans.getZ()), new Rotation3d()),
            10,
            1280,
            720,
            10);
    }

    public void periodic() {
        if (simCam != null && Robot.isSimulation()) simCam.processFrame(PositionManager.getInstance().getRobotPose());
        targets.clear();
        for (PhotonTrackedTarget t : this.getLatestResult().getTargets()) {
            targets.add(new Target(t, this));
        }
    }

    /* Setters */

    /**
     * Sets camera pair
     * MAKE SURE THE LEFT CAMERA IS THE PRIMARY CAMERA
     * DO NOT SET A PAIR FOR THE RIGHT CAMERA
     * @param c The camera object to pair with
     */
    public void setCameraPair(Camera c) {
        this.pair = c;
    }

    public void add(Target t) {
        this.targets.add(t);
    }

    public void addSim(SimVisionTarget t) {
        simCam.addSimVisionTarget(t);
    }

    public void clear() {
        this.targets.clear();
    }

    /* Getters */
    
    public Transform3d getTransform3d() {
        return trans;
    }

    public Camera getCameraPair() {
        return pair;
    }

    public double getPairDistance() {
        return trans.getTranslation().getDistance(pair.getTransform3d().getTranslation());
    }

    public ArrayList<Target> getTargets() {
        return this.targets;
    }

}
