package org.texastorque.subsystems;

import org.texastorque.Debug;
import org.texastorque.Input;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.sensors.TorqueCANCoder;
import org.texastorque.torquelib.util.TorqueMath;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Arm extends TorqueSubsystem implements Subsystems {
    public static class ArmPose {
        private static final double TELESCOPE_TOLERANCE = .5,
                ROTARY_TOLERANCE = Units.degreesToRadians(5), WRIST_TOLERANCE = .1;

        public final double telescopePose, wristPose;
        public final Rotation2d rotaryPose;

        public ArmPose(final double telescopePose, final Rotation2d rotaryPose,
                final double wristPose) {
            this.telescopePose = telescopePose;
            this.rotaryPose = rotaryPose;
            this.wristPose = wristPose;
        }

        public boolean atPose(final double telescopeReal, final Rotation2d rotaryReal,
                final double wristReal) {
            return Math.abs(telescopeReal - telescopePose) < TELESCOPE_TOLERANCE
                    && Math.abs(rotaryReal.minus(rotaryPose).getRadians()) < ROTARY_TOLERANCE
                    && Math.abs(wristReal - wristPose) < WRIST_TOLERANCE;
        }
    }

    /* @formatter:off
    *
    *                    90°
    *                     ↑
    *                0° ←  * → 180°
    *     HARDSTOP: 290 / ↓ \ HARDSTOP: 235
    *                    270°
    *
    *                   ___
    *                  |...|
    *                   | |
    *                   | |
    *                 | |-| |
    *                 |     |
    *                 |     | ___
    *                 |_____|   /
    *                /________ /
    */
    public static enum State {
        HIGH_FORWARDS(
            new ArmPose(19, Rotation2d.fromDegrees(40), -.55)),
        HIGH_BACKWARDS(
            new ArmPose(19, Rotation2d.fromDegrees(40), -.55)),  
        MID_FORWARDS(
            new ArmPose(10, Rotation2d.fromDegrees(5), -.2)),
        MID_BACKWARDS(
            new ArmPose(10, Rotation2d.fromDegrees(5), -.2)),
        INTAKE_FORWARDS(
            new ArmPose(0, Rotation2d.fromDegrees(180), -.55)),
        INTAKE_BACKWARDS(
            new ArmPose(0, Rotation2d.fromDegrees(180), -.55)),
        STOW(
            new ArmPose(10, Rotation2d.fromDegrees(90), 0));
        // @formatter:on

        public final ArmPose cubePose, conePose;

        private State(final ArmPose both) {
            this(both, both);
        }

        private State(final ArmPose cubePose, final ArmPose conePose) {
            this.cubePose = cubePose;
            this.conePose = conePose;
        }

        private State(final State other) {
            this(other.cubePose, other.conePose);
        }

        public ArmPose get() {
            return intake.isCubeMode() ? cubePose : conePose;
        }
    }

    private static volatile Arm instance;

    public static synchronized final Arm getInstance() {
        return instance == null ? instance = new Arm() : instance;
    }

    private final double ROTARY_ENCODER_OFFSET = .03681546, WRIST_OFFSET = 0.875,
            TELESCOPE_MIN_POSITION = 0, TELESCOPE_MAX_POSITION = 20,
            WRIST_MIN_POSITION = -0.55, WRIST_MAX_POSITION = -0,
            MAX_ROTARY_VOLTS = 12, MAX_TELESCOPE_VOLTS = 12, MIN_WRIST_VOLTS = .25, MAX_WRIST_VOLTS = 4;

    private final TorqueNEO rotary, telescope, wrist;
    private final PIDController rotatePID, telescopePID, wristPID;

    private final TorqueCANCoder rotaryEncoder;
    private final CANCoderConfiguration cancoderConfig;
    private final AbsoluteEncoder wristEncoder;

    private Rotation2d currentRotaryPose;

    private double currentTelescopePose, currentWristPose;

    private State state;

    public Arm() {
        rotary = new TorqueNEO(Ports.ARM_ROTARY);
        rotary.addFollower(Ports.ARM_ROTARY_2, false);
        rotary.setVoltageCompensation(12.6);
        rotary.setBreakMode(true);
        rotary.burnFlash();
        rotatePID = new PIDController(7, 0, 0);

        telescope = new TorqueNEO(Ports.TELESCOPE);
        telescope.setVoltageCompensation(12.6);
        telescope.setBreakMode(true);
        telescope.setCurrentLimit(80);
        // telescopeEncoder = telescope.getAbsoluteEncoder(Type.kDutyCycle);
        telescopePID = new PIDController(2, 0, 0);
        telescope.burnFlash();

        wrist = new TorqueNEO(Ports.WRIST);
        wrist.setVoltageCompensation(12.6);
        wrist.setCurrentLimit(30);
        wrist.setBreakMode(true);
        wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);
        wristPID = new PIDController(30, 0, 0);
        wrist.burnFlash();

        rotaryEncoder = new TorqueCANCoder(Ports.ARM_ROTARY_ENCODER);
        cancoderConfig = new CANCoderConfiguration();

        currentRotaryPose = new Rotation2d(rotary.getPosition());
        state = State.STOW;

        cancoderConfig.sensorCoefficient = 2 * Math.PI / 4096.0;
        cancoderConfig.unitString = "rad";
        cancoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        rotaryEncoder.configAllSettings(cancoderConfig);
    }

    public void setState(final State state) {
        this.state = state;
    }

    @Override
    public void initialize(TorqueMode mode) {
    }

    @Override
    public void update(TorqueMode mode) {
        Debug.log("state", state.toString());
        Debug.log("shift", Input.getInstance().getArmShift());
        updateRotary();
        updateTelescope();
        updateWrist();
    }

    private void updateRotary() {
        currentRotaryPose = Rotation2d
                .fromRadians(TorqueMath.constrain0to2PI(rotaryEncoder.getPosition() - ROTARY_ENCODER_OFFSET));

        double currentDegrees = currentRotaryPose.getDegrees();

        if (290 <= currentDegrees && currentDegrees <= 360)
            currentDegrees -= 360;

        double wantedDegrees = state.get().rotaryPose.getDegrees();

        if (290 <= wantedDegrees && wantedDegrees <= 360)
            wantedDegrees -= 360;

        else if (235 <= wantedDegrees && wantedDegrees <= 290)
            wantedDegrees = 235;

        Debug.log("current rotary degrees", currentDegrees);
        Debug.log("wanted rotary degrees", wantedDegrees);

        double volts = rotatePID.calculate(Math.toRadians(currentDegrees), Math.toRadians(wantedDegrees));
        // volts += Math.cos(Math.toRadians(wantedDegrees)) * 2;
        volts = TorqueMath.constrain(volts, MAX_ROTARY_VOLTS);

        Debug.log("rotaryVolts", volts);

        // rotary.setVolts(volts);
    }

    private void updateTelescope() {
        currentTelescopePose = telescope.getPosition();
        Debug.log("Current Telescope Pose", currentTelescopePose);

        double wantedPose = state.get().telescopePose;
        wantedPose = TorqueMath.constrain(wantedPose, TELESCOPE_MIN_POSITION, TELESCOPE_MAX_POSITION);
        Debug.log("Telescope Wants", wantedPose);

        double volts = telescopePID.calculate(currentTelescopePose, wantedPose);
        volts = TorqueMath.constrain(volts, MAX_TELESCOPE_VOLTS);

        Debug.log("Telescope Volts", volts);
        Debug.log("Telescope Current", telescope.getCurrent());

        // telescope.setVolts(volts);
    }

    private void updateWrist() {
        currentWristPose = wristEncoder.getPosition() - WRIST_OFFSET;

        double desiredPose = TorqueMath.constrain(state.get().wristPose, WRIST_MIN_POSITION, WRIST_MAX_POSITION);

        Debug.log("Wrist Desired Position", desiredPose);

        Debug.log("Current Wrist Clicks", currentWristPose);

        // Since the wrist needs an additional feedforward, we add one based off what
        // direction the volts are trying to go
        double volts = wristPID.calculate(currentWristPose, desiredPose);
        Debug.log("Wrist PID Volts", volts);
        // volts = TorqueMath.signum(volts) * Math.max(MIN_WRIST_VOLTS,
        // Math.abs(volts));
        volts = -TorqueMath.constrain(volts, MAX_WRIST_VOLTS); // PID needs to be inverted because of mechanical

        Debug.log("Wrist Volts", volts);

        // wrist.setVolts(volts);
    }
}
