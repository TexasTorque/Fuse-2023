/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution. For more details, see
 * ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.commands;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Intake;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;

public final class Score extends TorqueSequence implements Subsystems {
    public Score(final Arm.State armScoreState) {
        this(armScoreState, Arm.State.STOW);
    }

    public Score(final Arm.State armScoreState, final Arm.State armEndState) {
        addBlock(arm.yieldState(armScoreState));
        // addBlock(arm.waitUntilAtState()); telescope can get stuck, so just outtake
        // after a couple secs
        addBlock(new TorqueWaitTime(2.5));
        addBlock(intake.yieldState(Intake.State.OUTTAKE));
        addBlock(new TorqueWaitTime(.5));
        addBlock(intake.yieldState(Intake.State.OFF));
        addBlock(arm.yieldState(armEndState));
    }
}
