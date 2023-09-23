/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution. For more details, see
 * ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import org.texastorque.Debug;
import org.texastorque.Field;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.controllers.AutoLevelController;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.commands.TorqueContinuous;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueState;
import org.texastorque.torquelib.base.TorqueStatorSubsystem;
import org.texastorque.torquelib.sensors.TorqueNavXGyro;
import org.texastorque.torquelib.swerve.TorqueSwerveModule2022;
import org.texastorque.torquelib.swerve.TorqueSwerveModule2022.SwerveConfig;
import org.texastorque.torquelib.swerve.TorqueSwerveSpeeds;
import org.texastorque.torquelib.util.TorqueMath;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public final class Drivebase extends TorqueStatorSubsystem<Drivebase.State> implements Subsystems {
    public static enum State implements TorqueState {
        FIELD_RELATIVE(null), ROBOT_RELATIVE(null), XF(FIELD_RELATIVE), BALANCE(FIELD_RELATIVE);

        public final State parent;

        private State(final State parent) {
            this.parent = parent == null ? this : parent;
        }
    }

    public enum SpeedSetting {
        SLOW(.25), MID(.5), FAST(1.0), SEQ(1);

        private static final SpeedSetting[] vals = values();

        public double speed;

        private SpeedSetting(final double speed) {
            this.speed = speed;
        }

        public SpeedSetting shiftUp() {
            return vals[Math.min((this.ordinal() + 1), vals.length - 2)];
        }

        public SpeedSetting shiftDown() {
            return vals[Math.max((this.ordinal() - 1), 0)];
        }
    }

    public static class SpeedSequence {
        final double initSpeed, finalSpeed, duration, startTime, speedDeceleration;

        // Linearly decreases the speed every second for a duration of time
        public SpeedSequence(final SpeedSetting initSpeed, final SpeedSetting finalSpeed, final double duration) {
            this.initSpeed = initSpeed.speed;
            this.finalSpeed = finalSpeed.speed;
            this.duration = duration;
            speedDeceleration = (this.initSpeed - this.finalSpeed) / duration;
            startTime = Timer.getFPGATimestamp();
        }

        public double get() {
            return Math.max(initSpeed - speedDeceleration * (Timer.getFPGATimestamp() - startTime), finalSpeed);
        }
    }

    private static volatile Drivebase instance;

    public static final double WIDTH = Units.inchesToMeters(18), LENGTH = Units.inchesToMeters(21),
            MAX_VELOCITY = 4.522, MAX_ACCELERATION = 8.958, MAX_ANGULAR_VELOCITY = 2 * Math.PI,
            MAX_ANGULAR_ACCELERATION = 2 * Math.PI, WHEEL_DIAMETER = Units.inchesToMeters(4.0);

    public static final Pose2d INITIAL_POS = new Pose2d(0, 0, Rotation2d.fromRadians(0));

    /**
     * Standard deviations of model states. Increase these numbers to trust your
     * model's state
     * estimates less. This matrix is in the form [x, y, theta]ᵀ, with units in
     * meters and radians,
     * then meters.
     */
    private static final Vector<N3> STATE_STDS = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));

    /**
     * Standard deviations of the vision measurements. Increase these numbers to
     * trust global
     * measurements from vision less. This matrix is in the form [x, y, theta]ᵀ,
     * with units in
     * meters and radians.
     */
    private static final Vector<N3> VISION_STDS = VecBuilder.fill(0.1, 0.1, Units.degreesToRadians(10));

    public static SwerveModulePosition invertSwerveModuleDistance(final SwerveModulePosition pose) {
        return new SwerveModulePosition(-pose.distanceMeters, pose.angle);
    }

    public static synchronized final Drivebase getInstance() {
        return instance == null ? instance = new Drivebase() : instance;
    }

    private final Translation2d LOC_FL = new Translation2d(LENGTH / 2, -WIDTH / 2),
            LOC_FR = new Translation2d(LENGTH / 2, WIDTH / 2),
            LOC_BL = new Translation2d(-LENGTH / 2, -WIDTH / 2),
            LOC_BR = new Translation2d(-LENGTH / 2, WIDTH / 2);

    private final SwerveDriveKinematics kinematics;

    private final SwerveDrivePoseEstimator poseEstimator;
    public final Field2d fieldMap = new Field2d();

    private final TorqueSwerveModule2022 fl, fr, bl, br;

    private final TorqueNavXGyro gyro = TorqueNavXGyro.getInstance();

    private double lastRotationRadians;

    private final PIDController teleopOmegaController = new PIDController(.25 * Math.PI, 0, 0);

    private SwerveModuleState[] swerveStates;
    public TorqueSwerveSpeeds inputSpeeds = new TorqueSwerveSpeeds(0, 0, 0);

    public double requestedRotation = 0;

    public boolean isRotationLocked = true;

    public SpeedSetting speedSetting = SpeedSetting.FAST;

    public SpeedSequence speedSequence = new SpeedSequence(speedSetting, speedSetting, -1);

    private final AutoLevelController autoLevelController = new AutoLevelController();

    public boolean inTeleop = false;

    private Drivebase() {
        super(State.FIELD_RELATIVE);

        teleopOmegaController.enableContinuousInput(-Math.PI, Math.PI);
        lastRotationRadians = gyro.getRotation2d().getRadians();

        final SwerveConfig config = SwerveConfig.defaultConfig;

        config.maxVelocity = MAX_VELOCITY;
        config.maxAcceleration = MAX_ACCELERATION;
        config.maxAngularVelocity = MAX_ANGULAR_VELOCITY;
        config.maxAngularAcceleration = MAX_ANGULAR_ACCELERATION;

        fl = new TorqueSwerveModule2022("Front Left", Ports.FL_MOD, TorqueMath.constrain0to2PI(-2.90077720631102),
                config);
        fr = new TorqueSwerveModule2022("Front Right", Ports.FR_MOD, TorqueMath.constrain0to2PI(2.004908837378025),
                config);
        bl = new TorqueSwerveModule2022("Back Left", Ports.BL_MOD, TorqueMath.constrain0to2PI(-.607455164194107),
                config);
        br = new TorqueSwerveModule2022("Back Right", Ports.BR_MOD, TorqueMath.constrain0to2PI(1.4542108476), config);

        kinematics = new SwerveDriveKinematics(LOC_BL, LOC_BR, LOC_FL, LOC_FR);

        poseEstimator = new SwerveDrivePoseEstimator(kinematics, gyro.getHeadingCCW(),
                getModulePositions(), INITIAL_POS, STATE_STDS, VISION_STDS);

        swerveStates = new SwerveModuleState[4];
        for (int i = 0; i < swerveStates.length; i++)
            swerveStates[i] = new SwerveModuleState();

        SmartDashboard.putData("FIELD", fieldMap);
    }

    public boolean isState(State state) {
        return desiredState == state;
    }

    public boolean isAutoLevelDone() {
        return autoLevelController.isDone();
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        mode.onAuto(() -> {
            isRotationLocked = false;
            desiredState = State.ROBOT_RELATIVE;
        });

        mode.onTeleop(() -> {
            isRotationLocked = true;
            desiredState = State.FIELD_RELATIVE;
            inTeleop = true;
        });
    }

    public SwerveModulePosition[] getModulePositions() {
        return new SwerveModulePosition[] { invertSwerveModuleDistance(fl.getPosition()),
                invertSwerveModuleDistance(fr.getPosition()),
                invertSwerveModuleDistance(bl.getPosition()),
                invertSwerveModuleDistance(br.getPosition()) };
    }

    public void convertToFieldRelative() {
        inputSpeeds = inputSpeeds.toFieldRelativeSpeeds(gyro.getHeadingCCW());
    }

    @Override
    public final void update(final TorqueMode mode) {
        updateFeedback();

        if (desiredState == State.XF) {
            xFactor();
        } else {
            if (mode.isTeleop()) {
                inputSpeeds = inputSpeeds
                        .times(speedSetting == SpeedSetting.SEQ ? speedSequence.get()
                                : speedSetting.speed);

                calculateTeleop();
                convertToFieldRelative();
            } else if (desiredState == State.BALANCE) {
                inputSpeeds = autoLevelController.calculate();
                convertToFieldRelative();
            }

            swerveStates = kinematics.toSwerveModuleStates(inputSpeeds);

            SwerveDriveKinematics.desaturateWheelSpeeds(swerveStates, MAX_VELOCITY);

            if (inputSpeeds.hasZeroVelocity()) {
                preseveModulePositions();
            } else {
                final boolean useSmartMode = mode.isAuto();
                fl.setDesiredState(swerveStates[0], useSmartMode);
                fr.setDesiredState(swerveStates[1], useSmartMode);
                bl.setDesiredState(swerveStates[2], useSmartMode);
                br.setDesiredState(swerveStates[3], useSmartMode);
            }
        }

        autoLevelController.resetIf(desiredState != State.BALANCE);

        desiredState = desiredState.parent;
        Debug.log("Speed Shift State", speedSetting.toString());
        Debug.log("Speed Shift Value",
                speedSetting == SpeedSetting.SEQ ? speedSequence.get() : speedSetting.speed);
    }

    public void resetPose(final Pose2d pose) {
        poseEstimator.resetPosition(gyro.getHeadingCCW(), getModulePositions(), pose);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(final Rotation2d rotation) {
        resetPose(new Pose2d(poseEstimator.getEstimatedPosition().getTranslation(), rotation));
    }

    public void setAngle(final Rotation2d rotation) {
        gyro.setOffsetCW(rotation);
    }

    public void resetPose(final Translation2d translation) {
        resetPose(new Pose2d(translation, gyro.getHeadingCCW()));
    }

    public void resetGyro() {
        gyro.setOffsetCW(Rotation2d.fromRadians(0));
    }

    public double getGyroAngle() {
        return gyro.getHeadingCCW().getRadians();
    }

    public TorqueCommand setStateCommand(final State state) {
        return new TorqueContinuous(() -> setState(state));
    }

    public boolean isNotMoving() {
        return inputSpeeds.hasZeroVelocity();
    }

    private void updateFeedback() {
        poseEstimator.update(gyro.getHeadingCCW(), getModulePositions());

        fieldMap.setRobotPose(DriverStation.getAlliance() == DriverStation.Alliance.Blue
                ? poseEstimator.getEstimatedPosition()
                : Field.reflectPosition(poseEstimator.getEstimatedPosition()));

        Debug.log("Drivebase pose", getPose().toString());
    }

    private void preseveModulePositions() {
        fl.setDesiredState(new SwerveModuleState(0, swerveStates[0].angle));
        fr.setDesiredState(new SwerveModuleState(0, swerveStates[1].angle));
        bl.setDesiredState(new SwerveModuleState(0, swerveStates[2].angle));
        br.setDesiredState(new SwerveModuleState(0, swerveStates[3].angle));
    }

    private void xFactor() {
        fl.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        fr.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
        bl.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(135)));
        br.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    private void calculateTeleop() {
        final double realRotationRadians = gyro.getHeadingCCW().getRadians();

        if (isRotationLocked && !inputSpeeds.hasRotationalVelocity()
                && inputSpeeds.hasTranslationalVelocity()) {
            final double omega = teleopOmegaController.calculate(realRotationRadians, lastRotationRadians);
            inputSpeeds.omegaRadiansPerSecond = omega;
        } else
            lastRotationRadians = realRotationRadians;
    }
}
