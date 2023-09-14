package org.texastorque;

import org.texastorque.subsystems.*;

public interface Subsystems {
    public final Drivebase drivebase = Drivebase.getInstance();
    public final Arm arm = Arm.getInstance();
    public final Lights lights = Lights.getInstance();
    public final Intake intake = Intake.getInstance();
}
