/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequence;

import org.texastorque.Subsystems;
import org.texastorque.auto.commands.FollowPath;
import org.texastorque.auto.commands.Score;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Intake;
import org.texastorque.torquelib.auto.TorqueSequence;

public final class Flat2 extends TorqueSequence implements Subsystems {
    public Flat2() {
        addBlock(intake.yieldGamePiece(Intake.GamePiece.CONE));

        addBlock(new Score(Arm.State.HIGH).command());

        addBlock(intake.yieldGamePiece(Intake.GamePiece.CUBE));

        addBlock(new FollowPath("flat1", 2.5, 3.5));

        addBlock(new Score(Arm.State.HIGH).command());
    }
}