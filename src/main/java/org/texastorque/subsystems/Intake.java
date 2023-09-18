package org.texastorque.subsystems;

import org.texastorque.Debug;
import org.texastorque.Input;
import org.texastorque.Ports;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import org.texastorque.torquelib.control.TorqueRequestableTimeout;
import org.texastorque.torquelib.motors.TorqueNEO;

public class Intake extends TorqueSubsystem {
    public static enum GamePiece {
        CUBE, CONE;
    }

    public static enum State {
        OFF(-2), INTAKE(-5, -12), OUTTAKE(12);

        public final double cubeSpeed, coneSpeed;

        private State(final double cubeSpeed, final double coneSpeed) {
            this.cubeSpeed = cubeSpeed;
            this.coneSpeed = coneSpeed;
        }

        private State(final double both) {
            this(both, both);
        }

        public double getCubeSpeed() {
            return Intake.getInstance().isCubeMode() ? cubeSpeed : coneSpeed;
        }
    }

    private static volatile Intake instance;

    public static synchronized final Intake getInstance() {
        return instance == null ? instance = new Intake() : instance;
    }

    private final TorqueNEO rollers;

    private State desiredState, activeState;

    private GamePiece gamePieceMode;
    private final TorqueRequestableTimeout spikeTimeout = new TorqueRequestableTimeout();

    public Intake() {
        rollers = new TorqueNEO(Ports.WRIST_ROLLERS);
        rollers.setVoltageCompensation(12.6);
        rollers.setCurrentLimit(10);
        rollers.setBreakMode(false);
        desiredState = State.OFF;
        activeState = State.OFF;
        gamePieceMode = GamePiece.CONE;
    }

    public void setDesiredState(State state) {
        this.desiredState = state;
    }

    public boolean isConeMode() {
        return gamePieceMode == GamePiece.CONE;
    }

    public boolean isCubeMode() {
        return gamePieceMode == GamePiece.CUBE;
    }

    public void setGamePieceMode(GamePiece gamePieceMode) {
        this.gamePieceMode = gamePieceMode;
    }

    @Override
    public void initialize(TorqueMode mode) {
    }

    @Override
    public void update(TorqueMode mode) {
        Debug.log("rollers current", rollers.getCurrent());

        if (desiredState == State.INTAKE) {
            if (!spikeTimeout.get() && rollers.getCurrent() >= (isConeMode() ? 15 : 8)) {
                Input.getInstance().setDriverRumbleFor(.2);
                Input.getInstance().setOperatorRumbleFor(.2);
            }
        } else {
            spikeTimeout.set(1.0);
        }

        activeState = desiredState;

        rollers.setVolts(activeState.getCubeSpeed());

        desiredState = State.OFF;
    }
};
