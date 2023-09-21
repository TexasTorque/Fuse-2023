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
import org.texastorque.subsystems.Intake;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueSequenceRunner;
import org.texastorque.torquelib.auto.commands.TorqueWaitForSeconds;

public final class Any1 extends TorqueSequence implements Subsystems {
    public Any1() {
        addBlock(intake.yieldGamePiece(Intake.GamePiece.CONE));
        addBlock(new TorqueSequenceRunner(new Score(Arm.State.HIGH)));
        addBlock(new TorqueWaitForSeconds(1));
        addBlock(new FollowPath("taxi", 2.5, 3.5));
    }
}