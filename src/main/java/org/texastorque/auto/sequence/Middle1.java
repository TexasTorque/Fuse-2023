package org.texastorque.auto.sequence;

import org.texastorque.Subsystems;
import org.texastorque.auto.commands.FollowPath;
import org.texastorque.auto.commands.Score;
import org.texastorque.subsystems.*;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;

public class Middle1 extends TorqueSequence implements Subsystems {

    public Middle1() {
        addBlock(intake.yieldGamePiece(Intake.GamePiece.CONE));

        addBlock(new Score(Arm.State.MID).command());

        addBlock(new TorqueWaitTime(1)); // Wait for the arm to go back to stow

        addBlock(new FollowPath("balance-out", 3.5, 2.5));

        addBlock(new TorqueWaitTime(2));

        addBlock(new FollowPath("balance-in", 3.5, 2.5));

        addBlock(drivebase.setStateCommand(Drivebase.State.BALANCE));

    }

}
