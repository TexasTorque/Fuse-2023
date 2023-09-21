package org.texastorque.auto.sequence;

import org.texastorque.Subsystems;
import org.texastorque.auto.commands.FollowPath;
import org.texastorque.auto.commands.Score;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Intake;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueWaitTime;

public class Middle1 extends TorqueSequence implements Subsystems {

    public Middle1() {
        addBlock(intake.yieldGamePiece(Intake.GamePiece.CONE));

        addBlock(new Score(Arm.State.HIGH).command());

        addBlock(new FollowPath("balance-out", 1.5, 2.5));

        addBlock(new TorqueWaitTime(2));

        addBlock(new FollowPath("balance-in", 1.5, 2.5));
    }

}
