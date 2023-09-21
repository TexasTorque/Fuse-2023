package org.texastorque.auto.sequence;

import org.texastorque.Subsystems;
import org.texastorque.auto.routines.FollowPath;
import org.texastorque.auto.routines.Score;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Intake;
import org.texastorque.torquelib.auto.TorqueSequence;
import org.texastorque.torquelib.auto.commands.TorqueSequenceRunner;
import org.texastorque.torquelib.auto.commands.TorqueWaitForSeconds;

public class Middle1 extends TorqueSequence implements Subsystems {

    public Middle1() {
        addBlock(intake.yieldGamePiece(Intake.GamePiece.CONE));
        addBlock(new TorqueSequenceRunner(new Score(Arm.State.HIGH)));
        addBlock(new FollowPath("balance-out", 1.5, 2.5));
        addBlock(new TorqueWaitForSeconds(2));
        addBlock(new FollowPath("balance-in", 1.5, 2.5));
    }

}
