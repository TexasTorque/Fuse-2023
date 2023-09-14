/**
 * Copyright 2023 Texas Torque.
 *
 * This file is part of Torque-2023, which is not licensed for distribution.
 * For more details, see ./license.txt or write <jus@justusl.com>.
 */
package org.texastorque.subsystems;

import java.util.function.Supplier;
import org.texastorque.Ports;
import org.texastorque.Subsystems;
import org.texastorque.torquelib.base.TorqueMode;
import org.texastorque.torquelib.base.TorqueSubsystem;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;

public final class Lights extends TorqueSubsystem implements Subsystems {
    public static class Solid extends LightAction {
        private final Supplier<Color> color;

        public Solid(final Supplier<Color> color) {
            this.color = color;
        }

        @Override
        public void run(AddressableLEDBuffer buff) {
            for (int i = 0; i < buff.getLength(); i++)
                buff.setLED(i, color.get());
        }
    }

    private static abstract class LightAction {
        public abstract void run(AddressableLEDBuffer buff);
    }

    private static volatile Lights instance;

    private static final int LENGTH = 300;

    public static final synchronized Lights getInstance() {
        return instance == null ? instance = new Lights() : instance;
    }

    private final AddressableLED superstructureLEDs;

    private final AddressableLEDBuffer buff;

    private LightAction solidPurple = new Solid(() -> Color.kPurple), solidYellow = new Solid(() -> Color.kYellow);

    private Lights() {
        superstructureLEDs = new AddressableLED(Ports.LIGHTS_SUPERSTRUCTURE);
        superstructureLEDs.setLength(LENGTH);

        buff = new AddressableLEDBuffer(LENGTH);

        for (int i = 0; i < buff.getLength(); i++)
            buff.setLED(i, Color.kGreen);

        superstructureLEDs.setData(buff);
    }

    @Override
    public final void initialize(final TorqueMode mode) {
        superstructureLEDs.start();
    }

    public final LightAction getColor(final TorqueMode mode) {
        return intake.isConeMode() ? solidYellow : solidPurple;
    }

    @Override
    public final void update(final TorqueMode mode) {
        getColor(mode).run(buff);
        superstructureLEDs.setData(buff);
    }
}