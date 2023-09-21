/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution. For more details, see
 * ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.routines;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Intake;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueWaitForSeconds;

public final class Score extends TorqueSequence implements Subsystems {
    public Score(final Arm.State armState) {
        this(armState, .5);
    }

    public Score(final Arm.State armState, final double waitTime) {
        addBlock(arm.yieldState(armState));
        addBlock(arm.waitUntilAtState());
        addBlock(intake.yieldState(Intake.State.OUTTAKE));
        addBlock(new TorqueWaitForSeconds(.4));
        addBlock(intake.yieldState(Intake.State.OFF));
        addBlock(arm.yieldState(Arm.State.STOW));
    }
}
