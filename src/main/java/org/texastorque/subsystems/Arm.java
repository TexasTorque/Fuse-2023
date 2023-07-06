package org.texastorque.subsystems;

import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;

public class Arm extends TorqueSubsystem implements Subsystems {
    public static volatile Arm instance;

    public Arm() {}

    public static final synchronized Arm getInstance() {
        return instance == null ? instance = new Arm() : instance;
    }

    @Override
    public void initialize(TorqueMode mode) {
    }

    @Override
    public void update(TorqueMode mode) {


    }
}
