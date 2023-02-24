package frc.robot.Auto.SystemsCheck;

import edu.wpi.first.wpilibj2.command.Command;

public interface SubChecker {

    public String getName();

    public Command check(boolean safe);

    //public boolean isDone();
    
}
