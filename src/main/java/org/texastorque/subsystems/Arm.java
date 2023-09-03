package org.texastorque.subsystems;

import org.texastorque.Ports;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.motors.TorqueNEO;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends TorqueSubsystem {
    private static volatile Arm instance;


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
        HIGH(new ArmPose(0, Rotation2d.fromDegrees(20)),
            new ArmPose(0, Rotation2d.fromDegrees(20))), 
        MID(new ArmPose(0, Rotation2d.fromDegrees(10)),
            new ArmPose(0, Rotation2d.fromDegrees(10))),
        STOW(new ArmPose(0, Rotation2d.fromDegrees(200)),
            new ArmPose(0, Rotation2d.fromDegrees(200))), 
        INTAKE(new ArmPose(0, Rotation2d.fromDegrees(180)),
            new ArmPose(0, Rotation2d.fromDegrees(180)));
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
    private final AbsoluteEncoder rotateEncoder;

    private Rotation2d currentRotaryPose;
    private State state;


    public Arm() {
        rotate = new TorqueNEO(Ports.ARM);
        rotatePID = new PIDController(1, 0, 0);
        rotateEncoder = rotate.getAbsoluteEncoder(Type.kDutyCycle);

        currentRotaryPose = new Rotation2d(rotate.getPosition());
        state = State.STOW;
    }

    public void setState(final State state) {
        this.state = state;
    }

    @Override
    public void initialize(TorqueMode mode) {}

    @Override
    public void update(TorqueMode mode) {
        currentRotaryPose = Rotation2d.fromDegrees(rotateEncoder.getPosition());

        SmartDashboard.putNumber("arm::currentRotaryPose", currentRotaryPose.getRadians());

        rotate.setVolts(rotatePID.calculate(currentRotaryPose.getRadians(),
                state.get().rotaryPose.getRadians()));
    }

    public static synchronized final Arm getInstance() {
        return instance == null ? instance = new Arm() : instance;
    }
}
