/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution. For more details, see
 * ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Drivebase;
import org.texastorque.subsystems.Drivebase.SpeedSequence;
import org.texastorque.subsystems.Drivebase.SpeedSetting;
import org.texastorque.subsystems.Intake;
import org.texastorque.torquelib.base.TorqueInput;
import org.texastorque.torquelib.control.TorqueBoolSupplier;
import org.texastorque.torquelib.control.TorqueClickSupplier;
import org.texastorque.torquelib.control.TorqueRequestableTimeout;
import org.texastorque.torquelib.control.TorqueToggleSupplier;
import org.texastorque.torquelib.sensors.TorqueController;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.util.TorqueMath;

import edu.wpi.first.wpilibj.Timer;

public final class Input extends TorqueInput<TorqueController> implements Subsystems {
    private static volatile Input instance;

    private final static double DEADBAND = 0.125;

    public static final synchronized Input getInstance() {
        return instance == null ? instance = new Input() : instance;
    }

    private final TorqueBoolSupplier xFactor, resetGyro, high, mid, stow, gamePieceModeToggle, runIntake, runOuttake,
            shiftArmDirection, ground, highStow, doubleSub, slowlySlowDownClick,
            slowlySlowDownHold, lowDump, bucketHigh;

    private final TorqueRequestableTimeout driverTimeout, operatorTimeout;

    private Input() {
        driver = new TorqueController(0, .001);
        operator = new TorqueController(1);

        driverTimeout = new TorqueRequestableTimeout();
        operatorTimeout = new TorqueRequestableTimeout();

        xFactor = new TorqueToggleSupplier(driver::isXButtonDown);
        resetGyro = new TorqueClickSupplier(driver::isRightCenterButtonPressed);
        slowlySlowDownClick = new TorqueClickSupplier(driver::isLeftTriggerDown);
        slowlySlowDownHold = new TorqueBoolSupplier(driver::isLeftTriggerDown);

        high = new TorqueClickSupplier(operator::isYButtonDown);
        bucketHigh = new TorqueClickSupplier(operator::isDPADLeftDown);
        mid = new TorqueClickSupplier(operator::isBButtonDown);
        stow = new TorqueClickSupplier(operator::isAButtonDown);
        ground = new TorqueClickSupplier(() -> operator.isDPADDownDown());
        highStow = new TorqueClickSupplier(operator::isXButtonDown);
        doubleSub = new TorqueClickSupplier(operator::isDPADUpDown);
        lowDump = new TorqueClickSupplier(operator::isLeftCenterButtonDown);

        shiftArmDirection = new TorqueToggleSupplier(operator::isRightBumperDown);

        runIntake = new TorqueBoolSupplier(
                () -> operator.isRightTriggerDown() || operator.isDPADDownDown() || operator.isDPADUpDown());
        runOuttake = new TorqueBoolSupplier(operator::isLeftTriggerDown);

        gamePieceModeToggle = new TorqueToggleSupplier(operator::isLeftBumperDown);
    }

    public void update() {
        updateDrivebase();
        updateArm();
        updateIntake();

        if (drivebase.inTeleop && Timer.getMatchTime() == 10) {
            setDriverRumbleFor(2);
            setOperatorRumbleFor(2);
        }

        operator.setRumble(operatorTimeout.get());
        driver.setRumble(driverTimeout.get());
    }

    public void updateArm() {
        high.onTrue(() -> arm.setState(Arm.State.HIGH));
        bucketHigh.onTrue(() -> arm.setState(Arm.State.BUCKET_HIGH));
        mid.onTrue(() -> arm.setState(Arm.State.MID));
        stow.onTrue(() -> arm.setState(Arm.State.STOW));
        ground.onTrue(() -> arm.setState(Arm.State.INTAKE));
        highStow.onTrue(() -> arm.setState(Arm.State.HIGH_STOW));
        doubleSub.onTrue(() -> arm.setState(Arm.State.DOUBLE_SUB));
        lowDump.onTrue(()-> arm.setState(Arm.State.LOW_DUMP));

        arm.setRotaryAdjustment(-operator.getRightYAxis());
    }

    public void updateIntake() {
        runIntake.onTrue(() -> intake.setState(Intake.State.INTAKE));
        runOuttake.onTrue(() -> intake.setState(Intake.State.OUTTAKE));

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

        slowlySlowDownClick.onTrue(() -> {
            drivebase.speedSequence = new SpeedSequence(Drivebase.SpeedSetting.FAST,
                    Drivebase.SpeedSetting.SLOW, 1);
        });
        slowlySlowDownHold.onTrue(() -> {
            drivebase.speedSetting = SpeedSetting.SEQ;
        });

        if (!slowlySlowDownClick.get() && !slowlySlowDownHold.get())
            drivebase.speedSetting = Drivebase.SpeedSetting.FAST;

        final double xVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftYAxis(), DEADBAND)
                * Drivebase.MAX_VELOCITY;
        final double yVelocity = TorqueMath.scaledLinearDeadband(driver.getLeftXAxis(), DEADBAND)
                * Drivebase.MAX_VELOCITY;

        final double rotationVelocity = TorqueMath.scaledLinearDeadband(-driver.getRightXAxis(), DEADBAND)
                * Drivebase.MAX_ANGULAR_VELOCITY;

        drivebase.inputSpeeds = new TorqueSwerveSpeeds(xVelocity, yVelocity, rotationVelocity);
    }
}
