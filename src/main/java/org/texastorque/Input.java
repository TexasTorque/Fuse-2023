/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution. For more details, see
 * ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Intake;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueBoolSupplier;
import org.texastorque.torquelib.control.TorqueClickSupplier;
import org.texastorque.torquelib.control.TorqueRequestableTimeout;
import org.texastorque.torquelib.control.TorqueToggleSupplier;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.util.TorqueMath;

public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;

    private final static double DEADBAND = 0.125;

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }

    private final TorqueBoolSupplier xFactor, resetGyro, high, mid, stow, gamePieceModeToggle, runIntake, runOuttake,
            shiftArmDirection, ground, highStow;

    private final TorqueRequestableTimeout driverTimeout, operatorTimeout;

    private Input() {
        driver = new TorqueController(0, .001);
        operator = new TorqueController(1);

        driverTimeout = new TorqueRequestableTimeout();
        operatorTimeout = new TorqueRequestableTimeout();

        xFactor = new TorqueToggleSupplier(driver::isXButtonDown);
        resetGyro = new TorqueClickSupplier(driver::isRightCenterButtonPressed);

        high = new TorqueClickSupplier(operator::isYButtonDown);
        mid = new TorqueClickSupplier(operator::isBButtonDown);
        stow = new TorqueClickSupplier(operator::isAButtonDown);
        ground = new TorqueClickSupplier(() -> operator.isDPADDownDown());
        highStow = new TorqueClickSupplier(operator::isXButtonDown);

        shiftArmDirection = new TorqueToggleSupplier(operator::isRightBumperDown);

        runIntake = new TorqueBoolSupplier(() -> operator.isRightTriggerDown() || operator.isDPADDownDown());
        runOuttake = new TorqueBoolSupplier(operator::isLeftTriggerDown);

        gamePieceModeToggle = new TorqueToggleSupplier(operator::isLeftBumperDown);
    }

    public void update() {
        updateDrivebase();
        updateArm();
        updateIntake();

        operator.setRumble(operatorTimeout.get());
        driver.setRumble(driverTimeout.get());
    }
    public void updateArm() {
        high.onTrue(() -> arm.setDesiredState(Arm.State.HIGH));
        mid.onTrue(() -> arm.setDesiredState(Arm.State.MID));
        stow.onTrue(() -> arm.setDesiredState(Arm.State.STOW));
        ground.onTrue(() -> arm.setDesiredState(Arm.State.INTAKE));
        highStow.onTrue(() -> arm.setDesiredState(Arm.State.HIGH_STOW));
    }

    public void updateIntake() {
        runIntake.onTrue(() -> intake.setDesiredState(Intake.State.INTAKE));
        runOuttake.onTrue(() -> intake.setDesiredState(Intake.State.OUTTAKE));

        gamePieceModeToggle.onTrueOrFalse(() -> intake.setGamePieceMode(Intake.GamePiece.CONE),
                () -> intake.setGamePieceMode(Intake.GamePiece.CUBE));
    }

    public boolean isArmShift() {
        return shiftArmDirection.get();
    }

    public void setOperatorRumbleFor(final double duration) {
        operatorTimeout.set(duration);
    }

    public void setDriverRumbleFor(final double duration) {
        driverTimeout.set(duration);
    }

    private void updateDrivebase() {
        resetGyro.onTrue(() -> drivebase.resetGyro());
        xFactor.onTrue(() -> drivebase.setState(Drivebase.State.XF));

        final double xVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftYAxis(), DEADBAND)
                * Drivebase.MAX_VELOCITY;
        final double yVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftXAxis(), DEADBAND)
                * Drivebase.MAX_VELOCITY;

        final double rotationVelocity = TorqueMath.scaledLinearDeadband(-driver.getRightXAxis(), DEADBAND)
                * Drivebase.MAX_ANGULAR_VELOCITY;

        drivebase.inputSpeeds = new TorqueSwerveSpeeds(xVelocity, yVelocity, rotationVelocity);
    }
}
