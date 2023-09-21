package org.texastorque;

import org.texastorque.auto.AutoManager;
import org.texastorque.torquelib.base.TorqueRobotBase;

import com.pathplanner.lib.server.PathPlannerServer;

public final class Robot extends TorqueRobotBase implements Subsystems {
    public Robot() {
        super(Input.getInstance(), AutoManager.getInstance());

        addSubsystem(drivebase);
        addSubsystem(arm);
        addSubsystem(lights);
        addSubsystem(intake);

        Debug.initDashboard();
        PathPlannerServer.startServer(5811);
    }
}