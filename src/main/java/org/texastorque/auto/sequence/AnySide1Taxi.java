/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.auto.sequence;

import org.texastorque.Subsystems;
import org.texastorque.auto.routines.Drive;
import org.texastorque.auto.routines.Score;
import org.texastorque.subsystems.Arm;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueSequenceRunner;

public final class AnySide1Taxi extends TorqueSequence implements Subsystems {
    public AnySide1Taxi() {
        addBlock(new TorqueSequenceRunner(new Score(Arm.State.HIGH)));
        addBlock(new Drive("taxi", 2.5, 3.5));
    }
}