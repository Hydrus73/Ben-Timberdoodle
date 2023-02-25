package frc.robot.Drivetrain;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Sensors.Field.PositionManager;
import frc.robot.Sensors.Gyro.Gyro;
import frc.robot.Sensors.Vision.VisionManager;
import frc.robot.Auto.SystemsCheck.SubChecker;
import frc.robot.Auto.SystemsCheck.SystemsCheck;

public class DrivetrainSubsystem extends SubsystemBase implements SubChecker {

    /* MODULES */
    private final SwerveModule frontLeft = new SwerveModule(
        "Front Left",
        DriveConstants.frontLeftDriveID,
        DriveConstants.frontLeftTurnID,
        DriveConstants.frontLeftCancoderID,
        DriveConstants.frontLeftOffset
    );
    private final SwerveModule frontRight = new SwerveModule(
        "Front Right",
        DriveConstants.frontRightDriveID,
        DriveConstants.frontRightTurnID,
        DriveConstants.frontRightCancoderID,
        DriveConstants.frontRightOffset
    );;
    private final SwerveModule backLeft = new SwerveModule(
        "Back Left",
        DriveConstants.backLeftDriveID,
        DriveConstants.backLeftTurnID,
        DriveConstants.backLeftCancoderID,
        DriveConstants.backLeftOffset
    );;
    private final SwerveModule backRight = new SwerveModule(
        "Back Right",
        DriveConstants.backRightDriveID,
        DriveConstants.backRightTurnID,
        DriveConstants.backRightCancoderID,
        DriveConstants.backRightOffset
    );;

    /* MODULE MANAGEMENT */
    private final SwerveModule[] modules;
    private SwerveModuleState[] states = new SwerveModuleState[4];

    private final SwerveModulePosition[] modulePositions;

    private final SwerveDriveKinematics kinematics;
    private ChassisSpeeds speeds;

    /* ODOMETRY */
    private final SwerveDriveOdometry odometry;
    private final Field2d field = new Field2d();

    private double start = System.currentTimeMillis();

    public DrivetrainSubsystem() {        
        modules = new SwerveModule[]{frontLeft, frontRight, backLeft, backRight};
        speeds = new ChassisSpeeds(0.0, 0.0, 0.0);
        
        for (SwerveModule s : modules) {
            s.initEncoder();
        }

        modulePositions = new SwerveModulePosition[]{
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };

        kinematics = new SwerveDriveKinematics(
            new Translation2d(ModuleConstants.kModuleToCenter, ModuleConstants.kModuleToCenter),
            new Translation2d(ModuleConstants.kModuleToCenter, -ModuleConstants.kModuleToCenter),
            new Translation2d(-ModuleConstants.kModuleToCenter, ModuleConstants.kModuleToCenter),
            new Translation2d(-ModuleConstants.kModuleToCenter, -ModuleConstants.kModuleToCenter));

        /* ODOMETRY */
        // odometry = new SwerveDriveOdometry(kinematics, getRobotAngle(), modulePositions);
        odometry = new SwerveDriveOdometry(kinematics, new Rotation2d(), modulePositions);

        /* TELEMETRY */
        // Shuffleboard.getTab("Field").add(field);
            
        Shuffleboard.getTab("Field").addNumber("Gyro", () -> getRobotAngle().getDegrees());
    }

    /*******************
     * Runs every tick *
     *******************/
    @Override
    public void periodic() {
        //SmartDashboard.putNumber("Gyro", pigeon.getYaw());

        for (int i1 = 0; i1 < modules.length; i1++) {
            modulePositions[i1] = new SwerveModulePosition(modules[i1].getDistance(), modules[i1].getTurnAngle());
        }

        if (Robot.isReal() && !isMoving(0.1, 0.1)) {
            resetOdo(VisionManager.getInstance().getEstimatedPose());
        } else {
            odometry.update(getRobotAngle(), modulePositions);
        }

        if (Robot.isSimulation()) {
            Pose2d t = VisionManager.getInstance().getEstimatedPose();
            PositionManager.getInstance().getObject("Estimated Pose").setPose(
                t.getX(),
                t.getY(),
                Gyro.getInstance().getAngle()
            );
        }
        PositionManager.getInstance().setRobotPose(odometry.getPoseMeters());

    }

    /**
     * Gets the command to systems check the subsystem
     * @return The command to systems check the subsystem
     */
    public Command check(boolean safe) {
        if (!safe) return new InstantCommand();
        return new InstantCommand(() -> {
            boolean[] good = new boolean[4];
            double start = System.currentTimeMillis();
            for (SwerveModule s : modules) {
                s.rawControl(0.5, 0.5);
            }
            while (System.currentTimeMillis() - start < 2500) {}
            for (int i = 0; i < modules.length; i++) {
                SwerveModule s = modules[i];
                CANSparkMax[] motors = s.getMotors();
                if (motors[0].getEncoder().getVelocity() > 0.1 && motors[0].getEncoder().getVelocity() > 0.1) {
                    good[i] = true;
                }
                s.rawControl(0, 0);
            }
            SystemsCheck.setSystemStatus(this, true);
            for (boolean b : good) {
                if (!b) SystemsCheck.setSystemStatus(this, false);
            }
            stop();
        }, this);
    }

    /**
     * Drives the drivetrain
     * @param sp The desired ChassisSpeeds
     */
    public void drive(ChassisSpeeds sp) {
        double elapsed = System.currentTimeMillis() - start;
        this.speeds = sp;
        states = kinematics.toSwerveModuleStates(speeds, new Translation2d(0.0, 0.0));
        for (int i = 0; i < 4; i++) {
            modules[i].setState(states[i]);
            //modules[i].setState(new SwerveModuleState(0.0, Rotation2d.fromDegrees(0)));
        }
        if (Robot.isSimulation()) {
            Gyro.getInstance().setAngle(Gyro.getInstance().getAngle().getDegrees()+180+elapsed/1000*Units.radiansToDegrees(sp.omegaRadiansPerSecond));
        }
        start = System.currentTimeMillis();
    }

    /**
     * Gets the desired path command
     * @param p The name of the desired path
     * @return The created path
     */
    public Command getFollowCommand(String p) {
        field.getObject("Trajectory").setTrajectory(
            PathPlanner.loadPath(p, new PathConstraints(1, 1)));
        PathPlannerTrajectory traj = PathPlanner.loadPath(p, new PathConstraints(1, 1));
        Pose2d initPose = traj.getInitialPose();
        Translation2d initTranslation = initPose.getTranslation();
        Rotation2d initRotation = initPose.getRotation();
        Pose2d currentPose = getPose();
        Translation2d currentTranslation = currentPose.getTranslation();
        Rotation2d currentRotation = currentPose.getRotation();
        PathPlannerTrajectory toStart = PathPlanner.generatePath(new PathConstraints(1, 1), new PathPoint(currentTranslation, currentRotation), new PathPoint(initTranslation, initRotation));
        traj = (PathPlannerTrajectory) toStart.concatenate(traj);
        Command s = new SequentialCommandGroup(
            /*new InstantCommand(() -> {
                resetOdo(new Pose2d(
                    init.getX(),
                    init.getY(),
                    getRobotAngle()
                ));
            }),*/
            new PPSwerveControllerCommand(
                traj,
                this::getPose,
                this.kinematics, 
                new PIDController(1, 0, 0), 
                new PIDController(1, 0, 0), 
                new PIDController(1, 0, 0), 
                this::setStates, 
                this)
            );
        s.setName(p);
        return s;
    }

    /**
     * Gets the name of the Subsystem
     */
    public String getName() {
        return "Drivetrain";
    }

    /**
     * Gets the current field position of the robot
     * @return The current Pose2d of the robot
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * Gets the gyro rotation of the robot [-180, 180]
     * @return The Rotation2d of the robot
     */
    public Rotation2d getRobotAngle() {
        return Gyro.getInstance().getAngle();
    }

    /**
     * Gets the list of swerve module states
     * @return The list of swevre module states
     */
    public SwerveModuleState[] getStates() {
        return new SwerveModuleState[]{frontLeft.getState(), frontRight.getState(), backLeft.getState(), backRight.getState()};
    }

    // public void initGyro() {
    //     for (int i = 0; i < 100; i++) {System.out.println(i);}
    //     pigeon.setYaw(Vision.estimateGyro().getDegrees()+180);
    // }

    /**
     * Determines wether or not the bot is moving
     * @param sp The current chassis speeds
     * @param moveThreshold The threshold to determine movement
     * @param turnThreshold The threshold to determine rotation
     * @return Is the bot moving
     */
    private boolean isMoving(double moveThreshold, double turnThreshold) {
        ChassisSpeeds sp = speeds;
        return sp.vxMetersPerSecond > moveThreshold || sp.vyMetersPerSecond > moveThreshold || sp.omegaRadiansPerSecond > turnThreshold;
    }

    // /**
    //  * DO NOT USE - NOT NECESSARY
    //  * Resets distance traveled for all swerve modules
    //  * Used when reseting odometry
    //  */
    // private void resetModulePositions() {
    //     for (SwerveModule s : modules) {
    //         s.resetDistance();
    //     }
    // }

    /**
     * Resets odometry
     * @param p The position you want odometry to be at
     */
    private void resetOdo(Pose2d p) {
        //resetModulePositions();
        odometry.resetPosition(getRobotAngle(), modulePositions, p);
    }

    /**
     * Sets the individual state of a swerve module
     * @param s The desired state
     * @param i Index of the swerve module
     * [frontLeft, frontRight, backLeft, backRight]
     */
    public void setModuleState(SwerveModuleState s, int i) {
        modules[i].setState(s);
    }

    /**
     * Sets the raw states of the swerve modules
     * Used for pathing
     * @param s States to set
     */
    private void setStates(SwerveModuleState[] s) {
        drive(kinematics.toChassisSpeeds(s));
        /*for (int i = 0; i < 4; i++) {
            states[i] = s[i];
            modules[i].setState(states[i]);
        }*/
    }

    /**
     * Sets the drivetrain to point in an X position
     * Used to minimize movement
     * @param b Whether to put the drivetrain in an X or not
     */
    public void setX(boolean b) {
        if (b) {
            // -45 45 45 -45
            double[] a = new double[]{45, -45, -45, 45};
            for (int i = 0; i < modules.length; i++) {
                modules[i].setState(
                    new SwerveModuleState(0.0, Rotation2d.fromDegrees(a[i]))
                );
            }
        }
    }

    /**
     * Stops all drivetrain movement
     */
    public void stop() {
        this.drive(
            new ChassisSpeeds(0.0, 0.0, 0.0)
        );
    }

}