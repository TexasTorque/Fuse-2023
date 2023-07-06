/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution. For more details, see
 * ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import org.texastorque.controllers.PathAlignController.AlignState;
import org.texastorque.controllers.PathAlignController.GridState;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Drivebase.SpeedSetting;
import org.texastorque.subsystems.Hand.GamePiece;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueBoolSupplier;
import org.texastorque.torquelib.control.TorqueClickSupplier;
import org.texastorque.torquelib.control.TorqueRequestableTimeout;
import org.texastorque.torquelib.control.TorqueToggleSupplier;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.util.TorqueMath;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;

    private final static double DEADBAND = 0.125;

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }

    private final TorqueBoolSupplier xFactorToggle, resetGyroClick, wantsIntake, wantsOuttake,
            gamePieceModeToggle, slowMode;



    private Input() {
        driver = new TorqueController(0, .001);
        operator = new TorqueController(1);

        xFactorToggle = new TorqueToggleSupplier(driver::isXButtonDown);
        resetGyroClick = new TorqueClickSupplier(driver::isRightCenterButtonPressed);


        wantsIntake = new TorqueBoolSupplier(
                () -> operator.isRightTriggerDown() || driver.isRightTriggerDown());

        wantsOuttake = new TorqueBoolSupplier(
                () -> operator.isLeftCenterButtonDown() || driver.isLeftTriggerDown());


        gamePieceModeToggle = new TorqueToggleSupplier(
                () -> operator.isLeftBumperDown() || driver.isYButtonDown());

        slowMode = new TorqueToggleSupplier(driver::isAButtonDown);

    }



    public void update() {
        updateDrivebaseSpeeds();

        drivebase.setState(Drivebase.State.FIELD_RELATIVE);

        resetGyroClick.onTrue(() -> drivebase.resetGyro());

        drivebase.isRotationLocked = true;

        xFactorToggle.onTrue(() -> drivebase.setState(Drivebase.State.XF));

        gamePieceModeToggle.onTrueOrFalse(() -> hand.setGamePieceMode(GamePiece.CONE),
                () -> hand.setGamePieceMode(GamePiece.CUBE));

        // wantsIntake.onTrueOrFalse(() -> {
        //     intake.setState(Intake.State.INTAKE);
        // }, () -> {
        //     intake.setState(Intake.State.UP);
        // });

        // wantsOuttake.onTrue(() -> intake.setState(Intake.State.OUTAKE));
    }



    private void updateDrivebaseSpeeds() {
        SmartDashboard.putBoolean("slowMode", slowMode.get());
        drivebase.speedSetting = slowMode.get() ? SpeedSetting.SLOW : SpeedSetting.FAST;

        final double xVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftYAxis(), DEADBAND)
                * Drivebase.MAX_VELOCITY;
        final double yVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftXAxis(), DEADBAND)
                * Drivebase.MAX_VELOCITY;

        final double rotationVelocity =
                TorqueMath.scaledLinearDeadband(-driver.getRightXAxis(), DEADBAND)
                        * Drivebase.MAX_ANGULAR_VELOCITY;
        drivebase.inputSpeeds = new TorqueSwerveSpeeds(xVelocity, yVelocity, rotationVelocity);
    }
}
