package org.texastorque.subsystems;

import org.texastorque.Debug;
import org.texastorque.Input;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.auto.commands.TorqueWaitUntil;
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

    public static class ArmPosePair {
        public ArmPose cube, cone;

        public ArmPosePair(final ArmPose cube, final ArmPose cone) {
            this.cube = cube;
            this.cone = cone;
        }

        public ArmPosePair(final ArmPose both) {
            this(both, both);
        }

        public ArmPose get() {
            return intake.isConeMode() ? cone : cube;
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
        // FORWARD: ArmPosePair(cube, cone), BACKWARD!!
        HIGH(
            new ArmPosePair(
                new ArmPose(21, Rotation2d.fromDegrees(28), -.15),
                new ArmPose(18, Rotation2d.fromDegrees(31), -.2))),
        MID(
            new ArmPosePair(
                new ArmPose(0, Rotation2d.fromDegrees(18), -.12),
                new ArmPose(0, Rotation2d.fromDegrees(28), -.12)),
            new ArmPosePair(
                new ArmPose(13, Rotation2d.fromDegrees(167), -.5),
                new ArmPose(15, Rotation2d.fromDegrees(152), -.55))),
        INTAKE(
            new ArmPosePair(
                new ArmPose(9, Rotation2d.fromDegrees(215), -.38),
                new ArmPose(9, Rotation2d.fromDegrees(235), -.25))),
        STOW(
            new ArmPosePair(
                new ArmPose(-.8, Rotation2d.fromDegrees(232), -.04))),
        HIGH_STOW(
            new ArmPosePair(
                new ArmPose(0, Rotation2d.fromDegrees(135), 0))),
        MOVING_IN(
            new ArmPosePair(
                new ArmPose(0, Rotation2d.fromDegrees(-1), -.04)) 
        ), 
        DOUBLE_SUB(
            new ArmPosePair(
                new ArmPose(0, Rotation2d.fromDegrees(135), -.55),
                new ArmPose(1.8, Rotation2d.fromDegrees(138), -.52)
            )
        );
        // @formatter:on

        public final ArmPosePair forwards, backwards;

        private State(final ArmPosePair both) {
            this(both, both);
        }

        private State(final ArmPosePair cubePose, final ArmPosePair conePose) {
            this.forwards = cubePose;
            this.backwards = conePose;
        }

        private State(final State other) {
            this(other.forwards, other.backwards);
        }

        public ArmPosePair getPair() {
            return Input.getInstance().isArmShift() ? backwards : forwards;
        }

        public ArmPose get() {
            return getPair().get();
        }
    }

    private static volatile Arm instance;

    private static final double ARM_RESTRICTED_MIN = 235, ARM_RESTRICTED_MAX = 290;

    public static synchronized final Arm getInstance() {
        return instance == null ? instance = new Arm() : instance;
    }

    private final double ROTARY_ENCODER_OFFSET = .03681546, WRIST_OFFSET = 0.875,
            TELESCOPE_MIN_POSITION = 0, TELESCOPE_MAX_POSITION = 22.5,
            WRIST_MIN_POSITION = -0.55, WRIST_MAX_POSITION = -0,
            MAX_ROTARY_VOLTS = 12, MAX_TELESCOPE_VOLTS = 14, MAX_WRIST_VOLTS = 10,
            ROTARY_TOLERANCE = 7, TELESCOPE_TOLERANCE = 3, WRIST_TOLERANCE = .2, TELESCOPE_RATIO = 80/63;
    private final TorqueNEO rotary, telescope, wrist;

    private final PIDController rotatePID, telescopePID, wristPID;
    private final TorqueCANCoder rotaryEncoder;
    private final CANCoderConfiguration cancoderConfig;

    private final AbsoluteEncoder wristEncoder;

    private Rotation2d currentRotaryPose;

    private double currentTelescopePose, wantedTelescopePose, currentWristPose, wantedWristPose, currentRotaryDegrees,
            wantedRotaryDegrees, telescopeVelocity, telescopeDelta, rotaryAdjustment;

    private State desiredState;

    public Arm() {
        rotary = new TorqueNEO(Ports.ARM_ROTARY);
        rotary.addFollower(Ports.ARM_ROTARY_2, false);
        rotary.setVoltageCompensation(12.6);
        rotary.setBreakMode(true);
        rotary.setCurrentLimit(30);
        rotary.burnFlash();
        rotatePID = new PIDController(4, 0, 0);
        rotaryEncoder = new TorqueCANCoder(Ports.ARM_ROTARY_ENCODER);
        cancoderConfig = new CANCoderConfiguration();
        currentRotaryPose = new Rotation2d(rotary.getPosition());

        telescope = new TorqueNEO(Ports.TELESCOPE);
        telescope.setBreakMode(true);
        telescope.setCurrentLimit(40);
        telescopePID = new PIDController(2, 0, 0);
        telescope.burnFlash();

        wrist = new TorqueNEO(Ports.WRIST);
        wrist.setVoltageCompensation(12.6);
        wrist.setCurrentLimit(30);
        wrist.setBreakMode(true);
        wristEncoder = wrist.getAbsoluteEncoder(Type.kDutyCycle);
        wristPID = new PIDController(25, 0, 0);
        wrist.burnFlash();

        desiredState = State.STOW;

        cancoderConfig.sensorCoefficient = 2 * Math.PI / 4096.0;
        cancoderConfig.unitString = "rad";
        cancoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        rotaryEncoder.configAllSettings(cancoderConfig);
    }

    public boolean isAtState() {
        return Math.abs(currentRotaryDegrees - wantedRotaryDegrees) <= ROTARY_TOLERANCE
                && Math.abs(currentTelescopePose - wantedTelescopePose) <= TELESCOPE_TOLERANCE
                && Math.abs(currentWristPose - wantedWristPose) <= WRIST_TOLERANCE;
    }

    public boolean wantsDoubleSub() {
        return desiredState == State.DOUBLE_SUB;
    }

    public void setState(final State state) {
        this.desiredState = state;
    }

    public TorqueCommand yieldState(final State state) {
        return new TorqueExecute(() -> setState(state));
    }

    public TorqueCommand waitUntilAtState() {
        return new TorqueWaitUntil(this::isAtState);
    }

    public boolean isTelescopeAtPose() {
        return Math.abs(wantedTelescopePose - currentTelescopePose) < TELESCOPE_TOLERANCE;
    }

    public void setRotaryAdjustment(double setpointAdjustment) {
        rotaryAdjustment = setpointAdjustment;
    }

    @Override
    public void initialize(TorqueMode mode) {
    }

    @Override
    public void update(TorqueMode mode) {
        Debug.log("Arm State", desiredState.toString());

        double rotaryAngleDelta = Math.abs(wantedRotaryDegrees - currentRotaryDegrees);
        Debug.log("Rotary Angle Delta", rotaryAngleDelta);

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

        if (ARM_RESTRICTED_MAX <= currentRotaryDegrees && currentRotaryDegrees <= 360)
            currentRotaryDegrees -= 360;

        wantedRotaryDegrees = state.get().rotaryPose.getDegrees() + rotaryAdjustment * 3;

        if (ARM_RESTRICTED_MAX <= wantedRotaryDegrees && wantedRotaryDegrees <= 360)
            wantedRotaryDegrees -= 360;
        else if (ARM_RESTRICTED_MIN <= wantedRotaryDegrees && wantedRotaryDegrees <= ARM_RESTRICTED_MAX)
            wantedRotaryDegrees = ARM_RESTRICTED_MIN;

        Debug.log("Current Rotary Degrees", currentRotaryDegrees);
        Debug.log("Wanted Rotary Degrees", wantedRotaryDegrees);

        double volts = rotatePID.calculate(Math.toRadians(currentRotaryDegrees), Math.toRadians(wantedRotaryDegrees));
        volts += Math.cos(Math.toRadians(wantedRotaryDegrees)) * 1;
        volts = TorqueMath.constrain(volts, MAX_ROTARY_VOLTS);

        rotary.setVolts(volts);
    }

    private void updateTelescope(State state) {
        currentTelescopePose = telescope.getPosition() * TELESCOPE_RATIO;
        telescopeVelocity = telescope.getVelocity();

        Debug.log("Current Telescope Pose", currentTelescopePose);

        wantedTelescopePose = state.get().telescopePose * TELESCOPE_RATIO;

        wantedTelescopePose = TorqueMath.constrain(wantedTelescopePose, TELESCOPE_MIN_POSITION, TELESCOPE_MAX_POSITION);

        double volts = telescopePID.calculate(currentTelescopePose, wantedTelescopePose);
        volts = TorqueMath.constrain(volts, 6);

        // 🐐 code
        if (Math.abs(telescopeVelocity) <= 300 && Math.abs(telescopeDelta) >= 3) {
            long t = Math.round(Timer.getFPGATimestamp() * 3);
            if (t % 2 == 0) {
                volts = Math.signum(telescopeDelta) * 14;
            } else {
                // Lord woodie flowerz plz forgive me 4 dis code
                volts = telescopeDelta < 0 ? 0.001 : -4;
            }
        }
        // volts += Math.signum(volts) * 8;

        volts = TorqueMath.constrain(volts, MAX_TELESCOPE_VOLTS);

        telescope.setVolts(volts);
    }

    private void updateWrist(State state) {
        currentWristPose = wristEncoder.getPosition() - WRIST_OFFSET;

        wantedWristPose = TorqueMath.constrain(state.get().wristPose, WRIST_MIN_POSITION, WRIST_MAX_POSITION);

        Debug.log("Wrist Desired Position", wantedWristPose);

        Debug.log("Current Wrist Position", currentWristPose);

        // if (state == State.MOVING_IN) {
        // // double wantedRot = desiredState.get().rotaryPose.getDegrees();
        // if (currentRotaryDegrees >= ARM_RESTRICTED_MAX)
        // currentRotaryDegrees -= 360;
        // desiredPose = currentRotaryDegrees <= 90 ? WRIST_MIN_POSITION :
        // WRIST_MAX_POSITION;
        // // -.55 : 0
        // }

        // Since the wrist needs an additional feedforward, we add one based off what
        // direction the volts are trying to go
        double volts = wristPID.calculate(currentWristPose, wantedWristPose);

        // volts = TorqueMath.signum(volts) * Math.max(MIN_WRIST_VOLTS,
        // Math.abs(volts));

        // if ((currentRotaryDegrees <= 90) && (desiredState == State.HIGH ||
        // desiredState == State.MID)) volts -= 1.5;
        volts = -TorqueMath.constrain(volts, MAX_WRIST_VOLTS); // PID needs to be inverted because of mechanical

        // Debug.log("Wrist PID Volts", volts);

        if (desiredState == State.INTAKE && intake.isConeMode())
            volts = 1;

        wrist.setVolts(volts);
    }
}
