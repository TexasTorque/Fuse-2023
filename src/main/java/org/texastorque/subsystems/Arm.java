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
import edu.wpi.first.wpilibj.Timer;

public class Arm extends TorqueSubsystem implements Subsystems {
    public static class ArmPose {
        public final double telescopePose, wristPose;
        public final Rotation2d rotaryPose;

        public ArmPose(final double telescopePose, final Rotation2d rotaryPose,
                final double wristPose) {
            this.telescopePose = telescopePose;
            this.rotaryPose = rotaryPose;
            this.wristPose = wristPose;
        }

    }

    /* @formatter:off
    *
    *                    90°
    *                     ↑
    *                0° ←  * → 180°
    *     HARDSTOP: 315 / ↓ \ HARDSTOP: 235
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
        HIGH(
            new ArmPose(20, Rotation2d.fromDegrees(47), -.12),
            new ArmPose(20, Rotation2d.fromDegrees(135), -.48)),
        MID(
            new ArmPose(0, Rotation2d.fromDegrees(50), -.11),
            new ArmPose(12.5, Rotation2d.fromDegrees(140), -.55)),
        INTAKE(
            new ArmPose(4, Rotation2d.fromDegrees(230), -.28),
            new ArmPose(6.5, Rotation2d.fromDegrees(300), -.45)),
        STOW(
            new ArmPose(0, Rotation2d.fromDegrees(240), 0)),
        HIGH_STOW(
            new ArmPose(0, Rotation2d.fromDegrees(135), 0)
        ),
        MOVING_IN(
            new ArmPose(0, Rotation2d.fromDegrees(-1), 0)), 
        MOVING_MID(
            new ArmPose(10, Rotation2d.fromDegrees(-1), -.55) 
        );
        // @formatter:on

        public final ArmPose forwards, backwards;

        private State(final ArmPose both) {
            this(both, both);
        }

        private State(final ArmPose cubePose, final ArmPose conePose) {
            this.forwards = cubePose;
            this.backwards = conePose;
        }

        private State(final State other) {
            this(other.forwards, other.backwards);
        }

        public ArmPose get() {
            return Input.getInstance().isArmShift() ? backwards : forwards;
        }
    }

    private static volatile Arm instance;

    private static final double TELESCOPE_TOLERANCE = .5;

    public static synchronized final Arm getInstance() {
        return instance == null ? instance = new Arm() : instance;
    }

    private final double ROTARY_ENCODER_OFFSET = .03681546, WRIST_OFFSET = 0.875,
            TELESCOPE_MIN_POSITION = 0, TELESCOPE_MAX_POSITION = 20,
            WRIST_MIN_POSITION = -0.55, WRIST_MAX_POSITION = -0,
            MAX_ROTARY_VOLTS = 12, MAX_TELESCOPE_VOLTS = 14, MAX_WRIST_VOLTS = 8;

    private final TorqueNEO rotary, telescope, wrist;
    private final PIDController rotatePID, telescopePID, wristPID;

    private final TorqueCANCoder rotaryEncoder;
    private final CANCoderConfiguration cancoderConfig;
    private final AbsoluteEncoder wristEncoder;

    private Rotation2d currentRotaryPose;

    private double currentTelescopePose, wantedTelescopePose, currentWristPose, currentRotaryDegrees,
            wantedRotaryDegrees, telescopeVelocity;

    private State desiredState;

    double telescopeDelta = 0;

    public Arm() {
        rotary = new TorqueNEO(Ports.ARM_ROTARY);
        rotary.addFollower(Ports.ARM_ROTARY_2, false);
        rotary.setVoltageCompensation(12.6);
        rotary.setBreakMode(true);
        rotary.burnFlash();
        rotatePID = new PIDController(7, 0, 0);
        rotaryEncoder = new TorqueCANCoder(Ports.ARM_ROTARY_ENCODER);
        cancoderConfig = new CANCoderConfiguration();
        currentRotaryPose = new Rotation2d(rotary.getPosition());

        telescope = new TorqueNEO(Ports.TELESCOPE);
        telescope.disableVoltageCompensation();
        telescope.setBreakMode(true);
        telescope.setCurrentLimit(40);
        telescopePID = new PIDController(2, 0, 0);
        telescope.burnFlash();

        wrist = new TorqueNEO(Ports.WRIST);
        wrist.setVoltageCompensation(12.6);
        wrist.setCurrentLimit(30);
        wrist.setBreakMode(true);
        wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);
        wristPID = new PIDController(10, 0, 0);
        wrist.burnFlash();

        desiredState = State.STOW;

        cancoderConfig.sensorCoefficient = 2 * Math.PI / 4096.0;
        cancoderConfig.unitString = "rad";
        cancoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        rotaryEncoder.configAllSettings(cancoderConfig);
    }

    public void setDesiredState(final State state) {
        this.desiredState = state;
    }

    public boolean isTelescopeAtPose() {
        return Math.abs(wantedTelescopePose - currentTelescopePose) < TELESCOPE_TOLERANCE;
    }

    @Override
    public void initialize(TorqueMode mode) {
    }

    @Override
    public void update(TorqueMode mode) {
        Debug.log("armDesiredState", desiredState.toString());
        Debug.log("armShift", Input.getInstance().isArmShift());

        double rotaryAngleDelta = Math.abs(wantedRotaryDegrees - currentRotaryDegrees);
        Debug.log("rot delta", rotaryAngleDelta);

        telescopeDelta = wantedTelescopePose - currentTelescopePose;

        if (20 <= rotaryAngleDelta) {
            updateRotary(desiredState);
            updateTelescope(State.MOVING_IN);
            updateWrist(State.MOVING_IN);
        } else {
            updateRotary(desiredState);
            updateTelescope(desiredState);
            updateWrist(desiredState);
        }

    }

    private void updateRotary(State state) {
        currentRotaryPose = Rotation2d
                .fromRadians(TorqueMath.constrain0to2PI(rotaryEncoder.getPosition() - ROTARY_ENCODER_OFFSET));

        currentRotaryDegrees = currentRotaryPose.getDegrees();

        if (315 <= currentRotaryDegrees && currentRotaryDegrees <= 360)
            currentRotaryDegrees -= 360;

        wantedRotaryDegrees = state.get().rotaryPose.getDegrees();

        if (315 <= wantedRotaryDegrees && wantedRotaryDegrees <= 360)
            wantedRotaryDegrees -= 360;
        else if (235 <= wantedRotaryDegrees && wantedRotaryDegrees <= 315)
            wantedRotaryDegrees = 235;

        Debug.log("current rotary degrees", currentRotaryDegrees);
        Debug.log("wanted rotary degrees", wantedRotaryDegrees);

        double volts = rotatePID.calculate(Math.toRadians(currentRotaryDegrees), Math.toRadians(wantedRotaryDegrees));
        volts += Math.cos(Math.toRadians(wantedRotaryDegrees)) * 1;
        volts = TorqueMath.constrain(volts, MAX_ROTARY_VOLTS);

        Debug.log("rotaryVolts", volts);

        rotary.setVolts(volts);
    }

    private void updateTelescope(State state) {
        currentTelescopePose = telescope.getPosition();
        telescopeVelocity = telescope.getVelocity();

        Debug.log("Current Telescope Pose", currentTelescopePose);
        Debug.log("telescope velocity", telescopeVelocity);

        wantedTelescopePose = state.get().telescopePose;
        wantedTelescopePose = TorqueMath.constrain(wantedTelescopePose, TELESCOPE_MIN_POSITION, TELESCOPE_MAX_POSITION);
        Debug.log("Telescope Wants", wantedTelescopePose);

        double volts = telescopePID.calculate(currentTelescopePose, wantedTelescopePose);
        volts = TorqueMath.constrain(volts, 6);

        // 🐐 code
        if (Math.abs(telescopeVelocity) <= 300 && Math.abs(telescopeDelta) >= 3) {
            long t = Math.round(Timer.getFPGATimestamp() * 3);
            if (t % 2 == 0) {
                volts = Math.signum(telescopeDelta) * 14;
            } else {
                // Lord woodie flowerz plz forgive me 4 dis code
                volts = telescopeDelta < 0 ? 0.005 : -2;
            }
        }
        // volts += Math.signum(volts) * 8;

        volts = TorqueMath.constrain(volts, MAX_TELESCOPE_VOLTS);

        Debug.log("Telescope Volts", volts);
        Debug.log("Telescope Current", telescope.getCurrent());

        telescope.setVolts(volts);
    }

    private void updateWrist(State state) {
        currentWristPose = wristEncoder.getPosition() - WRIST_OFFSET;

        double desiredPose = TorqueMath.constrain(state.get().wristPose, WRIST_MIN_POSITION, WRIST_MAX_POSITION);

        Debug.log("Wrist Desired Position", desiredPose);

        Debug.log("Current Wrist Clicks", currentWristPose);

        if (state == State.MOVING_IN) {
            // double wantedRot = desiredState.get().rotaryPose.getDegrees();
            if (currentRotaryDegrees >= 290)
                currentRotaryDegrees -= 360;
            desiredPose = currentRotaryDegrees <= 90 ? -.55 : 0;
        }

        // Since the wrist needs an additional feedforward, we add one based off what
        // direction the volts are trying to go
        double volts = wristPID.calculate(currentWristPose, desiredPose);
        Debug.log("Wrist PID Volts", volts);
        // volts = TorqueMath.signum(volts) * Math.max(MIN_WRIST_VOLTS,
        // Math.abs(volts));
        volts = -TorqueMath.constrain(volts, MAX_WRIST_VOLTS); // PID needs to be inverted because of mechanical

        Debug.log("Wrist Volts", volts);

        wrist.setVolts(volts);
    }
}
