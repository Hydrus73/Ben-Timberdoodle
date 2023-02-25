package frc.robot.Drivetrain;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule extends SubsystemBase {
    
    private final AnalogEncoder cancoder;

    private final CANSparkMax turn;
    private final CANSparkMax drive;

    private final RelativeEncoder turnEnc;
    private final RelativeEncoder driveEnc;

    private final SparkMaxPIDController turnPID;
    private final SparkMaxPIDController drivePID;

    // private final SlewRateLimiter turnSlewRateLimiter = new SlewRateLimiter(1);
    // private final SlewRateLimiter driveSlewRateLimiter = new SlewRateLimiter(2.0, -Double.MAX_VALUE, 0.0);

    private final double offset;
    private double startTime = System.currentTimeMillis();

    private SwerveModuleState state;

    public SwerveModule(String name, int driveId, int turnId, int channel, double offset) {

        // Deg/s
        //turnSlewRateLimiter = new SlewRateLimiter();

        this.offset = offset;

        this.state = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));

        cancoder = new AnalogEncoder(channel);//new DutyCycleEncoder(new DigitalInput(channel));
        // cancoder.setPositionOffset(this.offset);

        turn = new CANSparkMax(turnId, MotorType.kBrushless);
        drive = new CANSparkMax(driveId, MotorType.kBrushless);

        turn.restoreFactoryDefaults();
        drive.restoreFactoryDefaults();

        turn.setSmartCurrentLimit(35);
        drive.setSmartCurrentLimit(35);

        drive.enableVoltageCompensation(12.0);
        turn.enableVoltageCompensation(12.0);

        turn.setInverted(true);
        drive.setInverted(false);

        turn.setIdleMode(IdleMode.kBrake);
        drive.setIdleMode(IdleMode.kBrake);

        turnEnc = turn.getEncoder();
        driveEnc = drive.getEncoder();

        driveEnc.setPosition(0.0);
        turnEnc.setPosition(0.0);

        turnPID = turn.getPIDController();
        drivePID = drive.getPIDController();

        turnPID.setP(ModuleConstants.turnP);
        turnPID.setI(ModuleConstants.turnI);
        turnPID.setD(ModuleConstants.turnD);

        drivePID.setP(ModuleConstants.driveP);
        drivePID.setI(ModuleConstants.driveI);
        drivePID.setD(ModuleConstants.driveD);
        drivePID.setFF(ModuleConstants.driveFF);

        turnEnc.setVelocityConversionFactor(360 / ModuleConstants.STEER_RATIO / 60);
        turnEnc.setPositionConversionFactor(360 / ModuleConstants.STEER_RATIO);
        driveEnc.setVelocityConversionFactor(ModuleConstants.WHEEL_CIRCUMPHRENCE / ModuleConstants.DRIVE_RATIO / 60);
        driveEnc.setPositionConversionFactor(ModuleConstants.WHEEL_CIRCUMPHRENCE / ModuleConstants.DRIVE_RATIO);
    
        Shuffleboard.getTab("Drivetrain").addNumber(name, () -> turnEnc.getPosition());

        final ShuffleboardLayout layout = Shuffleboard.getTab("Drivetrain").getLayout(name, BuiltInLayouts.kList)
            .withSize(2, 3);

        layout.addNumber("Target Angle", () -> state.angle.getDegrees())
            .withPosition(0, 0)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kTextView);
        layout.addNumber("Current Angle", () -> getCanCoderAngle().getDegrees())
            .withPosition(0, 1)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kTextView);
        layout.addNumber("Velocity", () -> state.speedMetersPerSecond)
            .withPosition(0, 2)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kTextView);
        layout.addNumber("CanCoder", () -> cancoder.getAbsolutePosition())
            .withPosition(0, 2)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kTextView);
            layout.addNumber("ENC Angle", () -> getTurnAngle().getDegrees())
            .withPosition(0, 1)
            .withSize(2, 1)
            .withWidget(BuiltInWidgets.kTextView);

        Shuffleboard.getTab("Pids").addNumber(name, () -> getCanCoderAngle().getDegrees());
    }

    @Override
    public void periodic() {
        double elapsed = System.currentTimeMillis() - startTime;
        SmartDashboard.putNumber("Elapsed Time", elapsed);
        if (Robot.isSimulation()) {
            driveEnc.setPosition(driveEnc.getPosition() + elapsed/1000*state.speedMetersPerSecond);
            turnEnc.setPosition(state.angle.getDegrees());
        }
        startTime = System.currentTimeMillis();
    }

    /**
     * Calculate the angle motor setpoint based on the desired angle and the current angle measurement
     * @return Returns the calculated delta angle
     */
    public double calculateAdjustedAngle(double targetAngle, double currentAngle) {

        double modAngle = currentAngle % 360;

        if (modAngle < 0.0) modAngle += 360;
        
        double newTarget = targetAngle + currentAngle - modAngle;

        if (targetAngle - modAngle > 180) newTarget -= 360;
        else if (targetAngle - modAngle < -180) newTarget += 360;

        return newTarget;

    }

    /**
     * Gets the current rotation of the module
     * @return Rotation2d of the current module
     */
    public Rotation2d getCanCoderAngle() {
        double unsignedAngle = (360 * (cancoder.getAbsolutePosition() - offset) % 360);
        if (unsignedAngle < 0) unsignedAngle+=360;
        return Rotation2d.fromDegrees(unsignedAngle-180);
    }

    /**
     * Gets the total distance driven
     * @return Distance driven (meters)
     */
    public double getDistance() {
        return driveEnc.getPosition();
    }

    /**
     * Gets the motors
     * @return CANSparkMax[]{drive, turn}
     */
    protected CANSparkMax[] getMotors() {
        return new CANSparkMax[]{drive, turn};
    }

    /**
     * Gets the current module state
     * @return The current module state
     */
    public SwerveModuleState getState() {
        return state;
    }

    /**
     * Gets the current module angle based on relative encoder
     * @return Rotation2d of the current module angle
     */
    public Rotation2d getTurnAngle() {

        double unsignedAngle = turnEnc.getPosition() % 360;

        if (unsignedAngle < 0) unsignedAngle += 360;

        return Rotation2d.fromDegrees(unsignedAngle-180);

    }

    /**
     * Initializes the relative encoder
     */
    public void initEncoder() {
        turnEnc.setPosition(getCanCoderAngle().getDegrees()-180);
    }

    /**
     * Enables raw open loop control of the module
     * @param speed Drive speed [-1, 1]
     * @param turn Turn speed [-1, 1]
     */
    public void rawControl(double speed, double turn) {
        this.turn.set(turn);
        this.drive.set(speed);
    }

    /**
     * Resets total distance driven
     * Used for resetting odometry
     */
    public void resetDistance() {
        driveEnc.setPosition(0.0);
    }

    /**
     * Sets the module to the desired state
     * @param in The desired state
     */
    public void setState(SwerveModuleState in) {

        state = new SwerveModuleState(in.speedMetersPerSecond, in.angle);

        state = SwerveModuleState.optimize(state, Rotation2d.fromDegrees(turnEnc.getPosition()%360));

        // drivePID.setReference(state.speedMetersPerSecond, ControlType.kVelocity);

        // drive.set(driveSlewRateLimiter.calculate(state.speedMetersPerSecond/4.0));
        drive.set(state.speedMetersPerSecond/4.0);

        //turnPID.setReference((turnEnc.getPosition() + delta % 360), ControlType.kPosition);
        // turnPID.setReference(
        //     calculateAdjustedAngle(state.angle.getDegrees(), 
        //     getTurnAngle().getDegrees()), ControlType.kPosition);
        turnPID.setReference(
            state.angle.getDegrees(), ControlType.kPosition);
    }
}
