package org.texastorque.subsystems;

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
        OFF(0), INTAKE(6), OUTTAKE(-6), CURRENT_SPIKE(6);

        public final double rollerSpeed;

        private State(final double rollerSpeed) {
            this.rollerSpeed = rollerSpeed;
        }

        public double getRollerSpeed() {
            return rollerSpeed;
        }
    }

    private static volatile Intake instance;

    private static final double SPIKE_CURRENT = 15.0;
    public static synchronized final Intake getInstance() {
        return instance == null ? instance = new Intake() : instance;
    }
    private final TorqueNEO rollers;

    private State state;

    private GamePiece gamePieceMode;
    private boolean hasSpiked = false;
    private final TorqueRequestableTimeout spikeTimeout = new TorqueRequestableTimeout();

    public Intake() {
        rollers = new TorqueNEO(Ports.WRIST_ROLLERS);
        rollers.setVoltageCompensation(12.6);
        rollers.setBreakMode(false);
        state = State.OFF;
        gamePieceMode = GamePiece.CONE;
    }

    public void setState(State state) {
        this.state = state;
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
        if (state == State.CURRENT_SPIKE) {
            if (!spikeTimeout.get() && rollers.getCurrent() >= SPIKE_CURRENT) {
                state = State.OFF;
            }
        } else {
            spikeTimeout.set(1.0);
        }

        // rollers.setVolts(state.getRollerSpeed());

        state = State.OFF;
    }
};
