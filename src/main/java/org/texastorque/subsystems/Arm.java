package org.texastorque.subsystems;

import org.texastorque.Ports;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Arm extends TorqueSubsystem {
    private static volatile Arm instance;

    public static class ArmPose {
        private static final double TELESCOPE_TOLERANCE = .5,
                ROTARY_TOLERANCE = Units.degreesToRadians(5);

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

    /* @formatter:off
    *
    *                    90
    *                     ↑
    *                0 ←  * → 180
    *     HARDSTOP: 290 / ↓ \ HARDSTOP: 235
    *                    270
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
            new ArmPose(0, Rotation2d.fromDegrees(40)),
            new ArmPose(0, Rotation2d.fromDegrees(40))), 
        MID(
            new ArmPose(0, Rotation2d.fromDegrees(5)),
            new ArmPose(0, Rotation2d.fromDegrees(5))),
        STOW(
            new ArmPose(0, Rotation2d.fromDegrees(225)),
            new ArmPose(0, Rotation2d.fromDegrees(225))), 
        INTAKE_FORWARD(
            new ArmPose(0, Rotation2d.fromDegrees(215)),
            new ArmPose(0, Rotation2d.fromDegrees(215))),
        INTAKE_BACKWARD(
            new ArmPose(0, Rotation2d.fromDegrees(280)),
            new ArmPose(0, Rotation2d.fromDegrees(280)));
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
            // when hand is made
            // return hand.isCubeMode() ? cubePose : conePose;
            return conePose;
        }
    }

    private final double ROTARY_ENCODER_OFFSET = .03681546, TELESCOPE_MIN = 0, TELESCOPE_MAX = 50;

    private final TorqueNEO rotary, telescope;
    private final PIDController rotatePID, telescopePID;
    private final TorqueCANCoder rotaryEncoder;
    private final CANCoderConfiguration cancoderConfig;
    private final AbsoluteEncoder telescopeEncoder;

    private Rotation2d currentRotaryPose;
    private double currentTelescopePose;
    private State state;

    public Arm() {
        rotary = new TorqueNEO(Ports.ARM_ROTARY);
        rotary.addFollower(Ports.ARM_ROTARY_2, false);
        rotary.setVoltageCompensation(12.6);
        rotary.setBreakMode(true);
        rotary.burnFlash();

        telescope = new TorqueNEO(Ports.TELESCOPE);
        telescope.setVoltageCompensation(12.6);
        telescope.setBreakMode(true);
        telescopeEncoder = telescope.getAbsoluteEncoder(Type.kDutyCycle);
        telescopePID = new PIDController(1, 0, 0);
        telescope.burnFlash();

        rotatePID = new PIDController(20, 1, 0);
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
        updateRotary();
        updateTelescope();
    }

    private void updateRotary() {
        currentRotaryPose = Rotation2d
                .fromRadians(TorqueMath.constrain0to2PI(rotaryEncoder.getPosition() - ROTARY_ENCODER_OFFSET));

        SmartDashboard.putString("arm::state", state.toString());

        double offsetRotaryPose = state.get().rotaryPose.getDegrees();

        if (290 <= offsetRotaryPose && offsetRotaryPose <= 360)
            offsetRotaryPose -= 360;
        else if (235 <= offsetRotaryPose && offsetRotaryPose <= 290)
            offsetRotaryPose = 235;

        double offsetCurrentPose = currentRotaryPose.getDegrees();

        if (290 <= offsetCurrentPose && offsetCurrentPose <= 360)
            offsetCurrentPose -= 360;

        SmartDashboard.putNumber("arm::offsetCurrentPose", offsetCurrentPose);

        SmartDashboard.putNumber("arm::offsetRotaryPose", offsetRotaryPose);

        double rotaryVolts = TorqueMath.constrain(rotatePID.calculate(Math.toRadians(offsetCurrentPose),
                Math.toRadians(offsetRotaryPose)), 12);

        SmartDashboard.putNumber("arm::rotaryVolts", rotaryVolts);

        rotary.setVolts(rotaryVolts);
    }

    private void updateTelescope() {
        currentTelescopePose = telescopeEncoder.getPosition();
        double pidVolts = telescopePID.calculate(currentTelescopePose, state.get().telescopePose);
        double telescopeVolts = TorqueMath.linearConstraint(pidVolts, currentTelescopePose, TELESCOPE_MIN,
        TELESCOPE_MAX);

        SmartDashboard.putNumber("arm::currentTelescopePose", currentTelescopePose);
        // telescope.setVolts(telescopeVolts);
    }

    public static synchronized final Arm getInstance() {
        return instance == null ? instance = new Arm() : instance;
    }
}
