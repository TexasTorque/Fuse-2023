/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequence;

import org.texastorque.Subsystems;
import org.texastorque.auto.routines.FollowPath;
import org.texastorque.auto.routines.Score;
import org.texastorque.subsystems.Arm;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueExecute;
import org.texastorque.torquelib.auto.commands.TorqueSequenceRunner;
import org.texastorque.torquelib.auto.commands.TorqueWaitForSeconds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

public final class Any1 extends TorqueSequence implements Subsystems {
    public Any1() {
        addBlock(
                new TorqueExecute(() -> drivebase.resetPose(new Pose2d(new Translation2d(1.88, 5), new Rotation2d()))));
        addBlock(new TorqueSequenceRunner(new Score(Arm.State.HIGH)));
        addBlock(new TorqueWaitForSeconds(2));
        addBlock(new FollowPath("taxi", 1, 1));
    }
}