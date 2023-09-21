package org.texastorque.auto;

import org.texastorque.auto.sequence.*;
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
        // addSequence(new Flat3());
        addSequence(new Flat2Line());
    }
}