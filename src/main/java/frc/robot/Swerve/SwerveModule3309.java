package frc.robot.Swerve;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import friarLib2.hardware.SwerveModule;
import friarLib2.math.CTREModuleState;
import friarLib2.utility.PIDParameters;

/**
 * Represents the 2023 revision of our swerve module.
 * 
 * <p>
 * This year's version features an improved powertrain for the drive motor
 * (reduced slippage through the use of 5mm GT2 belts) and a CTRE CANcoder
 * on the steering axis. In addition, the drivetrain has been geared up
 * to 23 ft/s (~7 m/s).
 * 
 * <p>
 * The CANcoder is used to dectect any slippage in the module's steering
 * axis. It is found by comparing the Falcon 500's integrated encoder value to
 * that of the CANcoder. If there is a difference, then the robot knows that
 * the belts have slipped and can alert the operators accordingly.
 * 
 * <p>
 * To zero the module, we use the CANcoder's absolute positioning to set
 * the Falcon encoder's value. The CANcoder's absolute reading when the
 * module is facing zero degrees is stored in the encoder's "custom slot" at
 * index zero. Since that slot only stores a 32 bit integer, the actual value
 * we store there is the magnet offest multiplied by 100. Such multiplication
 * allows us to gain two decimal places of precision (which is plenty for this
 * task) from that integer slot.
 */
public class SwerveModule3309 implements SwerveModule {
    /********** Constants **********/
    public static final double WHEEL_DIAMETER_INCHES = 3.8;
    public static final double DRIVE_GEAR_RATIO = (60. / 20.) * (18. / 32.) * (45. / 15.);
    public static final double STEERING_GEAR_RATIO_FALCON = (100. / 24.) * (48. / 16.); // Gear ratio between the Falcon's shaft and the steering axis
    public static final double STEERING_GEAR_RATIO_ENCODER = (1. / 1.); // Gear ratio between the CANcoder and the steering axis
    public static final double SLIP_THRESHOLD = 5; // Steering axis will be considered to have slipped if the difference between the Falcon and CANcoder's readings is greater than this value

    public static final PIDParameters DRIVE_PID_GAINS = new PIDParameters(0, "Swerve Drive PID", .1, 0.0007, 0.1);
    public static final PIDParameters STEERING_PID_GAINS = new PIDParameters(0, "Swerve Steering PID", .1, 0.002, 0);
    public static final double ABSOLUTE_MAX_DRIVE_SPEED = 7; // meters/sec

    public static final CurrentLimitsConfigs DRIVE_MOTOR_CURRENT_LIMIT = new CurrentLimitsConfigs()
            .withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentThreshold(60)
            .withSupplyTimeThreshold(0.5);

    /********** Member Variables **********/
    public String name; // Used for displaying values on SmartDashboard
    private final TalonFX driveMotor;
    private final TalonFX steeringMotor;
    private final CANcoder steeringEncoder;

    /** 
     * How many degrees the steering axis must be at (relative to its zeroed
     * position) to be facing forward on the robot. Modules on different
     * corners of a drivetrain will have different offsets
     */
    private final double steeringOffset;

    private double lastAngle = 0.0;

    /**
     * Construct a new swerve module
     * 
     * @param steeringOffset How many degrees the steering axis must be at 
     *     (relative to its zeroed position) to be facing forward on the robot.
     *     Modules on different corners of a drivetrain will have different 
     *     offsets
     * @param driveMotorID CAN ID for the module's drive motor
     * @param steeringMotorID CAN ID for the module's steering motor
     * @param encoderID CAN ID for the module's CANcoder
     * @param name The module's name (used when outputting to SmartDashboard)
     */
    public SwerveModule3309 (double steeringOffset, int driveMotorID, int steeringMotorID, int encoderID, String name) {
        this.name = name;
        driveMotor = new TalonFX(driveMotorID);
        steeringMotor = new TalonFX(steeringMotorID);
        configMotors();

        this.steeringOffset = steeringOffset;

        // Initialize the encoder
        // DO NOT configure the factory defaults. Doing so will reset the manually tuned
        // magnet offsets stored in the encoder's "custom slot"
        steeringEncoder = new CANcoder(encoderID);

        //TODO: @Josh Does not have, need to see if we use
//        steeringEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
//        steeringEncoder.configMagnetOffset(0);

        zeroSteering();
    }

    /**
     * Constructs a new swerve module
     * 
     * @param steeringOffset How many degrees the steering axis must be at 
     *     (relative to its zeroed position) to be facing forward on the robot.
     *     Modules on different corners of a drivetrain will have different 
     *     offsets
     * @param IDs The collection of CAN ID's for the module
     * @param name The module's name (used when outputting to SmartDashboard)
     */
    public SwerveModule3309 (double steeringOffset, SwerveCANIDs IDs, String name) {
        this(steeringOffset, IDs.driveMotorID, IDs.steeringMotorID, IDs.CANcoderID, name);
    }

    /**
     * Sets the motor PID values to those which will make the robot move the way we want.
     */
    public void configMotors () {
        //Set the configuration for all motors
        TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
        talonFXConfiguration.withCurrentLimits(DRIVE_MOTOR_CURRENT_LIMIT);
        talonFXConfiguration.withMotorOutput(new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake));

        //Configure PID
        DRIVE_PID_GAINS.configureMotorPID(driveMotor);
        STEERING_PID_GAINS.configureMotorPID(steeringMotor);

        //Apply configuration
        driveMotor.getConfigurator().apply(talonFXConfiguration);
        steeringMotor.getConfigurator().apply(talonFXConfiguration);
    }

    /**
     * Set the state of the swerve module. Credit to team 364 for CTREModuleState
     * 
     * @param state The new target state for the module
     */
    public void setState (SwerveModuleState state) {
        
        state = CTREModuleState.optimize(state, getState().angle);

        double velocity = Conversions.mpsToEncoderTicksPer100ms(state.speedMetersPerSecond);
        VelocityVoltage v = new VelocityVoltage(velocity);
        driveMotor.setControl(v);

        double angle = (Math.abs(state.speedMetersPerSecond) <= (ABSOLUTE_MAX_DRIVE_SPEED * 0.01)) ? lastAngle : state.angle.getDegrees(); // Prevent rotating module if speed is less than 1%. Prevents Jittering.
        PositionVoltage p = new PositionVoltage(Conversions.degreesToEncoderTicksFalcon(angle));
        steeringMotor.setControl(p);
        lastAngle = angle;
    }

    /**
     * Get the physical position of the module
     * 
     * @return The module's position
     */
    public SwerveModuleState getState () {
        return new SwerveModuleState(
            Conversions.encoderTicksPer100msToMps(driveMotor.getVelocity().getValue()),
            Rotation2d.fromDegrees(getSteeringDegreesFromFalcon())
        );
    }

    public SwerveModulePosition getPosition () {
        return new SwerveModulePosition(
            Conversions.encoderTicksPer100msToMps(driveMotor.getVelocity().getValue()),
            Rotation2d.fromDegrees(getSteeringDegreesFromFalcon())
        );
    }

    @Override
    public CANcoder GetCanCoder() {
        return steeringEncoder;
    }

    public void zeroPosition () {
        driveMotor.setPosition(0);
    }


    /**
     * @return If the belts for the steering axis have slipped
     */
    @Override
    public boolean steeringHasSlipped () {
        return Math.abs(
            getSteeringDegreesFromFalcon() -
            getSteeringDegreesFromEncoder()
            ) >= SLIP_THRESHOLD;
    }

    /**
     * Use the CANcoder (whoose belt will not slip under normal circumstances)
     * to reset the steering Falcon's intgrated encoder.
     */
    @Override
    public void zeroSteering (int encoderOffset) {

        double absolutePosition = steeringEncoder.getAbsolutePosition().getValue() - encoderOffset;
        double absolutePositionFalcon = Conversions.degreesToEncoderTicksFalcon(absolutePosition);
        double absolutePositionEncoder = Conversions.degreesToEncoderTicksCANcoder(absolutePosition);
        steeringMotor.setPosition(absolutePositionFalcon);
        steeringEncoder.setPosition(absolutePositionEncoder);
    }

    /**
     * Gets the value stored in the CANcoder's custom paramter slot zero,
     * divided by 100.
     * 
     * <p>
     * That slot, if configured correctly, should contain the absolute encoder
     * reading when the swerve module's steering axis is at zero degrees, 
     * multiplied by 100.
     * 
     * <p>
     * To configure it correctly, open OutlineViewer and, under the 
     * "SmartDahsboard" table, there should be a numeric field called 
     * "{module name} CANcoder absolute value". With the robot disabled, move
     * the steering axis of the module to be at the zero degrees position, then
     * note the dashboard value at that position. Finally, use Phoenix Tuner to
     * set the "Custom Param 0" field on the appropriate encoder (under the 
     * "Config" tab) to that dashboard value multiplied by 100.
     */
    private double getMagnetOffsetFromCANcoderSlot () {
        //TODO: Do we need this 2 second timeout?
        // Can do through other means?
        return ;
//        return steeringEncoder.configGetCustomParam(0, 2000) / 100.0;
    }

    /**
     * @return The position of the steering axis according to the Falcon's encoder
     */
    private double getSteeringDegreesFromFalcon () {
        return Conversions.encoderTicksToDegreesFalcon(steeringMotor.getPosition().getValue());
    }

    /**
     * @return The position of the steering axis according to the CANcoder
     */
    private double getSteeringDegreesFromEncoder () {
        return Conversions.encoderTicksToDegreesCANcoder(steeringEncoder.getPosition().getValue());
    }


    @Override
    public void outputToDashboard () {
        SmartDashboard.putNumber(name + " CANcoder absolute value", steeringEncoder.getAbsolutePosition().getValue());
        SmartDashboard.putNumber(name + " CANcoder raw value", steeringEncoder.getPosition().getValue());
        SmartDashboard.putNumber(name + " CANcoder degrees", getSteeringDegreesFromEncoder());
        SmartDashboard.putNumber(name + " Falcon degrees", getSteeringDegreesFromFalcon());
        SmartDashboard.putNumber(name + " Falcon raw value", steeringMotor.getPosition().getValue());
        SmartDashboard.putBoolean(name + " has slipped", steeringHasSlipped());
    }

    /**
     * Unit conversions for the swerve module
     */
    public static class Conversions {
        public static double mpsToEncoderTicksPer100ms (double mps) {
            double wheelDiameterMeters = Units.inchesToMeters(WHEEL_DIAMETER_INCHES);
            return mps * (1.0/(wheelDiameterMeters * Math.PI)) * DRIVE_GEAR_RATIO * (2048.0/1.0) * (1.0/10.0);
        }

        public static double encoderTicksPer100msToMps (double encoderTicksPer100ms) {
            return encoderTicksPer100ms / mpsToEncoderTicksPer100ms(1);
        }

        public static double degreesToEncoderTicksFalcon (double degrees) {
            return degrees * (2048.0 / 360.0) * STEERING_GEAR_RATIO_FALCON;
        }

        public static double encoderTicksToDegreesFalcon (double encoderTicks) {
            return encoderTicks / degreesToEncoderTicksFalcon(1);
        }

        public static double degreesToEncoderTicksCANcoder (double degrees) {
            return degrees * STEERING_GEAR_RATIO_ENCODER;
        }

        public static double encoderTicksToDegreesCANcoder (double encoderTicks) {
            return encoderTicks / degreesToEncoderTicksCANcoder(1);
        }
    }

    


}