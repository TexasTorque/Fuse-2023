/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque;

import org.texastorque.torquelib.swerve.TorqueSwerveModule2022.SwervePorts;

public final class Ports {
    // CAN Swerve
    public static final SwervePorts FL_MOD = new SwervePorts(3, 4, 10);
    public static final SwervePorts FR_MOD = new SwervePorts(5, 6, 11);
    public static final SwervePorts BL_MOD = new SwervePorts(1, 2, 9);
    public static final SwervePorts BR_MOD = new SwervePorts(7, 8, 12);

    public static final int WRIST = 13;
    public static final int INTAKE = 14;
    public static final int WRIST_ENCODER = 15;
}
