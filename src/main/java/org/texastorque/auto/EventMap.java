package org.texastorque.auto;

import java.util.HashMap;
import java.util.Map;

import org.texastorque.Subsystems;
import org.texastorque.subsystems.Arm;
import org.texastorque.subsystems.Intake;
import org.texastorque.torquelib.auto.TorqueCommand;
import org.texastorque.torquelib.auto.commands.TorqueExecute;

public final class EventMap implements Subsystems {

    public static Map<String, TorqueCommand> get() {
        final Map<String, TorqueCommand> map = new HashMap<String, TorqueCommand>();

        map.put("intake-down-1", new TorqueExecute(() -> {
            arm.setState(Arm.State.INTAKE);
            intake.setState(Intake.State.INTAKE);
        }));

        map.put("stow-1", new TorqueExecute(() -> {
            arm.setState(Arm.State.STOW);
            intake.setState(Intake.State.OFF);
        }));

        map.put("cube-high", new TorqueExecute(() -> {
            arm.setState(Arm.State.HIGH);
        }));

        map.put("score-high", new TorqueExecute(() -> {
            intake.setState(Intake.State.OUTTAKE);
        }));

        map.put("stow-2", new TorqueExecute(() -> {
            arm.setState(Arm.State.STOW);
            intake.setState(Intake.State.OFF);
        }));

        map.put("intake-down-2", new TorqueExecute(() -> {
            arm.setState(Arm.State.INTAKE);
            intake.setState(Intake.State.INTAKE);
        }));


        map.put("stow-2", new TorqueExecute(() -> {
            arm.setState(Arm.State.STOW);
            intake.setState(Intake.State.OFF);
        }));

        map.put("cube-MID", new TorqueExecute(() -> {
            arm.setState(Arm.State.MID);
        }));

        map.put("score-mid", new TorqueExecute(() -> {
            intake.setState(Intake.State.OUTTAKE);
        }));

        map.put("stow-3", new TorqueExecute(() -> {
            arm.setState(Arm.State.STOW);
            intake.setState(Intake.State.OFF);
        }));

        return map;
    }
}
