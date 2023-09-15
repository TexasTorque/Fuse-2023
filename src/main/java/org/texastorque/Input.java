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
import org.texastorque.torquelib.control.TorqueToggleSupplier;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.util.TorqueMath;

public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;

    private final static double DEADBAND = 0.125;

    public static final synchronized Input getInstance() { return instance == null ? instance = new Input() : instance; }

    private final TorqueBoolSupplier xFactor, resetGyro, high, mid, stow, gamePieceModeToggle, runIntake, runOuttake, shiftArmDirection, ground;

    private TorqueController debugController = new TorqueController(2);

    private Input() {
        driver = new TorqueController(0, .001);
        operator = new TorqueController(1);

        xFactor = new TorqueToggleSupplier(driver::isXButtonDown);
        resetGyro = new TorqueClickSupplier(driver::isRightCenterButtonPressed);

        high = new TorqueClickSupplier(operator::isYButtonDown);
        mid = new TorqueClickSupplier(operator::isBButtonDown);
        stow = new TorqueClickSupplier(operator::isAButtonDown);
        ground = new TorqueClickSupplier(operator::isDPADDownDown);

        shiftArmDirection = new TorqueToggleSupplier(operator::isLeftBumperDown);

        // runIntake = new TorqueBoolSupplier(operator::isRightTriggerDown);
        runIntake = new TorqueBoolSupplier(() -> operator.isRightTriggerDown() || operator.isDPADDownDown());
        runOuttake = new TorqueBoolSupplier(operator::isLeftTriggerDown);

        gamePieceModeToggle = new TorqueToggleSupplier(operator::isRightBumperDown);
    }

    public void update() {
        updateDrivebase();
        // updateArm();
        updateArmDebug();
        updateIntake();
    }

    public void updateArm() {
        arm.debugMode = false;

        high.onTrue(() -> arm.setState(shiftArmDirection.get() ? Arm.State.HIGH_FORWARDS : Arm.State.HIGH_BACKWARDS));
        mid.onTrue(() -> arm.setState(shiftArmDirection.get() ? Arm.State.MID_FORWARDS : Arm.State.MID_BACKWARDS));
        stow.onTrue(() -> arm.setState(Arm.State.STOW));

        ground.onTrue(() -> arm.setState(shiftArmDirection.get() ? Arm.State.INTAKE_FORWARDS : Arm.State.INTAKE_BACKWARDS));
    }

    public void updateArmDebug() {
        arm.debugMode = true;

        arm.overrideRotary = debugController.getRightYAxis() * 3;

        arm.overrideTele = debugController.getLeftYAxis() * 3;

        if (debugController.isDPADUpDown())
            arm.overrideWrist = 3;
        else if (debugController.isDPADDownDown())
            arm.overrideWrist = -3;
        else
            arm.overrideWrist = 0;
    }

    public void updateIntake() {
        runIntake.onTrue(() -> intake.setState(Intake.State.INTAKE));
        runOuttake.onTrue(() -> intake.setState(Intake.State.OUTTAKE));

        gamePieceModeToggle.onTrueOrFalse(() -> intake.setGamePieceMode(Intake.GamePiece.CONE), () -> intake.setGamePieceMode(Intake.GamePiece.CUBE));
    }

    private void updateDrivebase() {
        resetGyro.onTrue(() -> drivebase.resetGyro());
        xFactor.onTrue(() -> drivebase.setState(Drivebase.State.XF));

        final double xVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftYAxis(), DEADBAND) * Drivebase.MAX_VELOCITY;
        final double yVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftXAxis(), DEADBAND) * Drivebase.MAX_VELOCITY;

        final double rotationVelocity = TorqueMath.scaledLinearDeadband(-driver.getRightXAxis(), DEADBAND) * Drivebase.MAX_ANGULAR_VELOCITY;

        drivebase.inputSpeeds = new TorqueSwerveSpeeds(xVelocity, yVelocity, rotationVelocity);
    }
}
