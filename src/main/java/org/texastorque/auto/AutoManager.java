package org.texastorque.auto;

import org.texastorque.auto.sequence.Any1;
import org.texastorque.auto.sequence.Flat2;
import org.texastorque.auto.sequence.Flat3;
import org.texastorque.auto.sequence.Middle1;
import org.texastorque.torquelib.auto.TorqueAutoManager;

public final class AutoManager extends TorqueAutoManager {
    private static volatile AutoManager instance;

    /**
     * Get the AutoManager instance
     *
     * @return AutoManager
     */
    public static final synchronized AutoManager getInstance() {
        return instance == null ? instance = new AutoManager() : instance;
    }

    @Override
    public final void init() {
        addSequence(new Any1());
        addSequence(new Middle1());
        addSequence(new Flat2());
        addSequence(new Flat3());
    }
}