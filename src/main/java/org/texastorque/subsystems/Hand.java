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

public class Hand extends TorqueSubsystem implements Subsystems {
    private static volatile Hand instance;

    private final static double WRIST_UP = 0, WRIST_LEFT = -90, WRIST_RIGHT = 90;

    public static enum GamePiece {
        CUBE, CONE;
    }

    public static class HandPose {
        public final double wristPose;
        public final double intakeSpeed;

        public HandPose(final double wristPose, final double intakeSpeed) {
            this.wristPose = wristPose;
            this.intakeSpeed = intakeSpeed;
        }
    }

    public static enum State {
        // @formatter:off
        UP(new HandPose(WRIST_UP, 1), new HandPose(WRIST_UP, .5)), 
        LEFT(new HandPose(WRIST_LEFT, 1)), 
        RIGHT(new HandPose(WRIST_RIGHT, 1));
        // @formatter:ON


        public final HandPose cubePose;
        public final HandPose conePose;

        private State(final HandPose both) {
            this(both, both);
        }

        private State(final HandPose cubePose, final HandPose conePose) {
            this.cubePose = cubePose;
            this.conePose = conePose;
        }

        public HandPose get() {
            return hand.isCubeMode() ? cubePose : conePose;
        }
    }

    private GamePiece mode;
    private State desiredState, activeState;
    private boolean runIntake;

    private final TorqueNEO wrist, intake;
    private final TorqueCANCoder wristEncoder;
    private final PIDController wristPoseController;


    public Hand() {
        mode = GamePiece.CUBE;

        desiredState = State.UP;
        runIntake = false;

        wrist = new TorqueNEO(Ports.WRIST);
        intake = new TorqueNEO(Ports.INTAKE);

        wrist.setVoltageCompensation(12.6);
        wrist.setBreakMode(true);
        wrist.burnFlash();

        intake.setVoltageCompensation(12.6);
        intake.setBreakMode(false);
        intake.burnFlash();

        wristEncoder = new TorqueCANCoder(Ports.WRIST_ENCODER);
        final CANCoderConfiguration cancoderConfig = new CANCoderConfiguration();
        cancoderConfig.sensorCoefficient = 2 * Math.PI / 4096.0;
        cancoderConfig.unitString = "rad";
        cancoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
        cancoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        wristEncoder.configAllSettings(cancoderConfig);

        wristPoseController = new PIDController(0.1, 0, 0);

    }

    public void setGamePieceMode(GamePiece mode) {
        this.mode = mode;
    }

    public boolean isCubeMode() {
        return mode == GamePiece.CUBE;
    }

    public boolean isConeMode() {
        return mode == GamePiece.CONE;
    }

    public boolean runningIntake() {
        return runIntake;
    }

    public void setIntake(boolean runIntake) {
        this.runIntake = runIntake;
    }


    @Override
    public void initialize(TorqueMode mode) {}

    @Override
    public void update(TorqueMode mode) {
        activeState = desiredState;
         
        wrist.setVolts(wristPoseController.calculate(wristEncoder.getAbsolutePosition(), activeState.get().wristPose));
        intake.setVolts(runIntake ? activeState.get().intakeSpeed : 0);

        if (mode.isTeleop()) runIntake = false;
    }

    public static final synchronized Hand getInstance() {
        return instance == null ? instance = new Hand() : instance;
    }
}
