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
        OFF(-2), INTAKE(-12), OUTTAKE(12), CURRENT_SPIKE(-12), HOLD_PIECE(-.5);

        public final double rollerSpeed;

        private State(final double rollerSpeed) {
            this.rollerSpeed = rollerSpeed;
        }

        public double getRollerSpeed() {
            return rollerSpeed;
        }
    }

    private static volatile Intake instance;

    private static final double SPIKE_CURRENT = 15;

    public static synchronized final Intake getInstance() {
        return instance == null ? instance = new Intake() : instance;
    }

    private final TorqueNEO rollers;

    private State desiredState, activeState;

    private GamePiece gamePieceMode;
    private boolean hasSpiked = false;
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
        Debug.log("intakeDesiredState", desiredState.toString());
        Debug.log("intakeActiveState", activeState.toString());

        if (desiredState == State.CURRENT_SPIKE || activeState == State.HOLD_PIECE && desiredState != State.OUTTAKE) {
            // The spike timeout is because there will be a current spike when the motor
            // starts moving so it waits a second for the current to stabalize down
            if (!spikeTimeout.get() && rollers.getCurrent() >= SPIKE_CURRENT) {
                // activeState = State.HOLD_PIECE;
                Input.getInstance().setDriverRumbleFor(.2);
                Input.getInstance().setOperatorRumbleFor(.2);
                hasSpiked = true;
            }
        } else {
            spikeTimeout.set(1.0);
            hasSpiked = false;
        }

        if (hasSpiked)
            desiredState = State.OFF;

        // activeState = hasSpiked ? State.HOLD_PIECE : desiredState;
        activeState = desiredState;

        rollers.setVolts(activeState.getRollerSpeed());

        desiredState = State.OFF;
    }
};
