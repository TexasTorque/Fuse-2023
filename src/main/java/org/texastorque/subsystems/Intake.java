package org.texastorque.subsystems;

import org.texastorque.Debug;
import org.texastorque.Input;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.commands.TorqueRun;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.control.TorqueRequestableTimeout;
import org.texastorque.torquelib.motors.TorqueNEO;

public class Intake extends TorqueStatorSubsystem<Intake.State> implements Subsystems {
    public static enum GamePiece {
        CUBE, CONE;
    }

    public static enum State implements TorqueState {
        OFF(-3), INTAKE(-5, -12), OUTTAKE(12);

        public final double cubeSpeed, coneSpeed;

        private State(final double cubeSpeed, final double coneSpeed) {
            this.cubeSpeed = cubeSpeed;
            this.coneSpeed = coneSpeed;
        }

        private State(final double both) {
            this(both, both);
        }

        public double get() {
            return Intake.getInstance().isCubeMode() ? cubeSpeed : coneSpeed;
        }
    }

    private static volatile Intake instance;

    public static synchronized final Intake getInstance() {
        return instance == null ? instance = new Intake() : instance;
    }

    private final TorqueNEO rollers;

    private GamePiece gamePieceMode;

    private final TorqueRequestableTimeout spikeTimeout = new TorqueRequestableTimeout();
    public Intake() {
        super(State.OFF, State.OFF);
        rollers = new TorqueNEO(Ports.WRIST_ROLLERS);
        rollers.setVoltageCompensation(12.6);
        rollers.setCurrentLimit(10);
        rollers.setBreakMode(false);
        gamePieceMode = GamePiece.CONE;
    }

    public boolean isActive() {
        return desiredState != State.OFF;
    }

    public boolean isConeMode() {
        return gamePieceMode == GamePiece.CONE;
    }

    public boolean isCubeMode() {
        return gamePieceMode == GamePiece.CUBE;
    }

    public void setGamePieceMode(final GamePiece gamePieceMode) {
        this.gamePieceMode = gamePieceMode;
    }

    @Override
    public void initialize(final TorqueMode mode) {
    }

    public TorqueCommand yieldGamePiece(final GamePiece gamePieceMode) {
        return new TorqueRun(() -> setGamePieceMode(gamePieceMode));
    }

    @Override
    public void update(final TorqueMode mode) {
        Debug.log("Rollers Current", rollers.getCurrent());

        if (desiredState == State.INTAKE) {
            if (!spikeTimeout.get() && rollers.getCurrent() >= (isConeMode() ? 12 : 8)) {
                Input.getInstance().setDriverRumbleFor(.2);
                Input.getInstance().setOperatorRumbleFor(.2);
            }
        } else {
            spikeTimeout.set(1.0);
        }

        rollers.setVolts(desiredState.get());

        if (mode.isTeleop())
            desiredState = State.OFF;
    }
};
