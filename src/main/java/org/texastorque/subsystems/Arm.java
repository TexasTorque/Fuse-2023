package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import org.texastorque.torquelib.sensors.TorqueCANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class Arm extends TorqueSubsystem implements Subsystems {
    public static volatile Arm instance;

    public static class ArmPose {
        private static final double TELESCROPE_TOLERANCE = .2,
                ROTARY_TOLERANCE = Units.degreesToRadians(5);

        public boolean autoReadyToScore = false;

        public final double telescopePose;
        public final Rotation2d rotaryPose;

        public ArmPose(final double telescopePose, final Rotation2d rotaryPose) {
            this.telescopePose = telescopePose;
            this.rotaryPose = rotaryPose;
        }

        public boolean atPose(final double telescopeReal, final Rotation2d rotaryReal) {
            return Math.abs(telescopeReal - telescopePose) < TELESCROPE_TOLERANCE
                    && Math.abs(rotaryReal.minus(rotaryPose).getRadians()) < ROTARY_TOLERANCE;
        }
    }

    public static enum State {
        // @formatter:off
        HOME(new ArmPose(0, new Rotation2d(0))), 
        INTAKE_FRONT(new ArmPose(0, new Rotation2d(0))),
        INTAKE_BACK(new ArmPose(0, new Rotation2d(0))),
        SCORE_HIGH(new ArmPose(0, new Rotation2d(0))),
        SCORE_MID(new ArmPose(0, new Rotation2d(0))),
        SCORE_LOW(new ArmPose(0, new Rotation2d(0)));
        // @formatter:on

        public final ArmPose cubePose;
        public final ArmPose conePose;

        private State(final ArmPose both) {
            this(both, both);
        }

        private State(final ArmPose cubePose, final ArmPose conePose) {
            this.cubePose = cubePose;
            this.conePose = conePose;
        }

        public ArmPose get() {
            return hand.isCubeMode() ? cubePose : conePose;
        }
    }

    private State desiredState, activeState;
    private final TorqueNEO telescope, rotary;
    private final TorqueCANCoder rotaryEncoder;
    private final PIDController telescopePID, rotaryPID;

    public Arm() {
        desiredState = State.HOME;

        telescope = new TorqueNEO(Ports.TELESCOPE);
        rotary = new TorqueNEO(Ports.ROTARY);

        rotaryEncoder = new TorqueCANCoder(Ports.WRIST_ENCODER);
        final CANCoderConfiguration cancoderConfig = new CANCoderConfiguration();
        cancoderConfig.sensorCoefficient = 2 * Math.PI / 4096.0;
        cancoderConfig.unitString = "rad";
        cancoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        rotaryEncoder.configAllSettings(cancoderConfig);

        telescopePID = new PIDController(0, 0, 0);
        rotaryPID = new PIDController(0, 0, 0);
    }

    public void setState(final State state) {
        desiredState = state;
    }


    @Override
    public void initialize(TorqueMode mode) {}

    @Override
    public void update(TorqueMode mode) {
        activeState = desiredState;

        telescope.setVolts(
                telescopePID.calculate(telescope.getPosition(), activeState.get().telescopePose));

        rotary.setVolts(calulateRotary());

    }

    private double calulateRotary() {
        return rotaryPID.calculate(rotaryEncoder.getPosition(),
                activeState.get().rotaryPose.getRadians());
    }

    public static final synchronized Arm getInstance() {
        return instance == null ? instance = new Arm() : instance;
    }
}
