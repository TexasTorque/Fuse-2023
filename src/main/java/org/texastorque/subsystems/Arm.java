package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class Arm extends TorqueSubsystem {

    public static class ArmPose {
        private static final double TELESCOPE_TOLERANCE = .6,
                ROTARY_TOLERANCE = Units.degreesToRadians(10);

        public final double telescopePose;
        public final Rotation2d rotaryPose;

        public ArmPose(final double telescopePose, final Rotation2d rotaryPose) {
            this.telescopePose = telescopePose;
            this.rotaryPose = rotaryPose;
        }

        public boolean atPose(final double telescopeReal, final Rotation2d rotaryReal) {
            return Math.abs(telescopeReal - telescopePose) < TELESCOPE_TOLERANCE
                    && Math.abs(rotaryReal.minus(rotaryPose).getRadians()) < ROTARY_TOLERANCE;
        }
    }

    public static enum State {
        // @formatter:off
        HIGH(new ArmPose(2.5, Rotation2d.fromDegrees(205)),
            new ArmPose(0, Rotation2d.fromDegrees(228))), 
        MID(new ArmPose(2.5, Rotation2d.fromDegrees(205)),
            new ArmPose(0, Rotation2d.fromDegrees(228))),
        STOW(new ArmPose(2.5, Rotation2d.fromDegrees(205)),
            new ArmPose(0, Rotation2d.fromDegrees(228))), 
        INTAKE(new ArmPose(2.5, Rotation2d.fromDegrees(205)),
            new ArmPose(0, Rotation2d.fromDegrees(228)));
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
            // return hand.isCubeMode() ? cubePose : conePose;
            return conePose;
        }
    }

    private final TorqueNEO rotate;
    private final PIDController rotatePID;
    private final DutyCycleEncoder rotateEncoder;
    private State state;
    

    public Arm() {
        rotate = new TorqueNEO(Ports.ARM);
        rotatePID = new PIDController(1, 0, 0);
        rotateEncoder = new DutyCycleEncoder(Ports.ARM_ENCODER);
        state = State.STOW;
    }

    @Override
    public void initialize(TorqueMode mode) {}

    @Override
    public void update(TorqueMode mode) {
        rotate.setVolts(rotatePID.calculate(rotate.getPosition(), state.get().rotaryPose.getRadians()));
    }
}
