/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution. For more details, see
 * ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Intake;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueBoolSupplier;
import org.texastorque.torquelib.control.TorqueClickSupplier;
import org.texastorque.torquelib.control.TorqueToggleSupplier;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.util.TorqueMath;

public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;

    private final static double DEADBAND = 0.125;

    private final TorqueBoolSupplier xFactor, resetGyro, high, mid, stow, groundIntake,
            gamePieceModeToggle, runIntake, runOuttake, shiftArmDirection;

    private Input() {
        driver = new TorqueController(0, .001);
        operator = new TorqueController(1);

        xFactor = new TorqueToggleSupplier(driver::isXButtonDown);
        resetGyro = new TorqueClickSupplier(driver::isRightCenterButtonPressed);

        high = new TorqueClickSupplier(operator::isYButtonDown);
        mid = new TorqueClickSupplier(operator::isBButtonDown);
        stow = new TorqueClickSupplier(operator::isAButtonDown);
        groundIntake = new TorqueBoolSupplier(operator::isDPADDownDown);
        shiftArmDirection = new TorqueToggleSupplier(operator::isLeftBumperDown);

        runIntake = new TorqueBoolSupplier(operator::isRightTriggerDown);
        runOuttake = new TorqueBoolSupplier(operator::isLeftTriggerDown);

        gamePieceModeToggle = new TorqueToggleSupplier(operator::isRightBumperDown);
    }

    public void update() {
        updateDrivebase();
        updateArm();
        updateIntake();
    }

    private void updateDrivebase() {
        resetGyro.onTrue(() -> drivebase.resetGyro());
        xFactor.onTrue(() -> drivebase.setState(Drivebase.State.XF));

        final double xVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftYAxis(), DEADBAND)
                * Drivebase.MAX_VELOCITY;
        final double yVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftXAxis(), DEADBAND)
                * Drivebase.MAX_VELOCITY;

        final double rotationVelocity =
                TorqueMath.scaledLinearDeadband(-driver.getRightXAxis(), DEADBAND)
                        * Drivebase.MAX_ANGULAR_VELOCITY;

        drivebase.inputSpeeds = new TorqueSwerveSpeeds(xVelocity, yVelocity, rotationVelocity);
    }

    public void updateArm() {
        high.onTrue(() -> arm.setState(
                shiftArmDirection.get() ? Arm.State.HIGH_FORWARDS : Arm.State.HIGH_BACKWARDS));
        mid.onTrue(() -> arm.setState(
                shiftArmDirection.get() ? Arm.State.MID_FORWARDS : Arm.State.MID_BACKWARDS));
        stow.onTrue(() -> arm.setState(Arm.State.STOW));

        groundIntake.onTrue(() -> arm.setState(
                shiftArmDirection.get() ? Arm.State.INTAKE_FORWARDS : Arm.State.INTAKE_BACKWARDS));

        arm.tempTeleVolts = operator.getLeftYAxis() * 3;
    }

    public void updateIntake() {
        groundIntake.onTrue(() -> intake.setState(Intake.State.INTAKE));

        runIntake.onTrue(() -> intake.setState(Intake.State.INTAKE));
        runOuttake.onTrue(() -> intake.setState(Intake.State.OUTTAKE));

        gamePieceModeToggle.onTrueOrFalse(() -> intake.setGamePieceMode(Intake.GamePiece.CONE),
                () -> intake.setGamePieceMode(Intake.GamePiece.CUBE));
    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}
