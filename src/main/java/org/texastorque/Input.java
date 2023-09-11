/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution. For more details, see
 * ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import org.texastorque.subsystems.Arm;
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

    private final TorqueBoolSupplier xFactor, resetGyro, high, mid, stow, intake;

    private Input() {
        driver = new TorqueController(0, .001);
        operator = new TorqueController(1);

        xFactor = new TorqueToggleSupplier(driver::isXButtonDown);
        resetGyro = new TorqueClickSupplier(driver::isRightCenterButtonPressed);

        high = new TorqueToggleSupplier(operator::isYButtonDown);
        mid = new TorqueToggleSupplier(operator::isBButtonDown);
        stow = new TorqueToggleSupplier(operator::isAButtonDown);
        intake = new TorqueToggleSupplier(driver::isRightTriggerDown);
    }


    public void update() {
        updateDrivebase();
        updateArm();
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
        high.onTrue(() -> arm.setState(Arm.State.HIGH));
        mid.onTrue(() -> arm.setState(Arm.State.MID));
        stow.onTrue(() -> arm.setState(Arm.State.STOW));
        intake.onTrue(() -> arm.setState(Arm.State.INTAKE));
    }

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }
}
