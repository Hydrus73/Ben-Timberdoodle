package frc.robot.Auto.SystemsCheck;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class SystemsCheck extends CommandBase {

    private final SubChecker[] t;

    private static final HashMap<SubChecker, Boolean> a = new HashMap<>();

    private boolean hasRun = false;

    private final ArrayList<Command> commands = new ArrayList<>();

    public SystemsCheck(SubChecker... t) {
        this.t = t;

        ShuffleboardTab tab = Shuffleboard.getTab("Systems Check");

        tab.addBoolean("Has Run", () -> hasRun);

        for (SubChecker s : t) {
            a.put(s, false);

            tab.addBoolean(s.getName(), () -> a.get(s));
        }

    }

    @Override
    public void initialize() {
        for (SubChecker s : t) {
            //double start = System.currentTimeMillis();
            Command cmd = s.check(true);
            commands.add(cmd);
            cmd.schedule();
        }
        
    }

    @Override
    public void execute() {
    }

    @Override
    public boolean isFinished() {
        boolean allFinished = true;
        for (Command c : commands) {
            if (!c.isFinished()) {
                allFinished = false;
            }
        }
        return allFinished;
    }

    @Override
    public void end(boolean interr) {
        hasRun = true;
    }

    public static void setSystemStatus(SubChecker s, Boolean status) {
        a.replace(s, status);
    }
    
}