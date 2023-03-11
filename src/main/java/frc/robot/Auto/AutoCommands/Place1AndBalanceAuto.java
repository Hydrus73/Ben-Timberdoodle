// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.Drivetrain.DrivetrainSubsystem;

public class Place1AndBalanceAuto {
  ArrayList<PathPlannerTrajectory> trajectories;
  HashMap<String, Command> events;
  Command fullAutoCommand;
  /** Creates a new Place1AndBalanceAuto. */
  public Place1AndBalanceAuto(DrivetrainSubsystem drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    trajectories = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("Place1AndBalance", new PathConstraints(Constants.AutoConstants.MAX_VELOCITY, Constants.AutoConstants.MAX_ACCELERATION));
    events.put("PlaceStartCone", new PlaceCone());
    events.put("Balance", new AutoLevel(drivetrain));
    
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      () -> drivetrain.getPose(),
      (Pose2d pose) -> drivetrain.resetOdo(pose),
      drivetrain.kinematics,
      new PIDConstants(Constants.AutoConstants.TRANSLATIONkP, Constants.AutoConstants.TRANSLATIONkI, Constants.AutoConstants.TRANSLATIONkD),
      new PIDConstants(Constants.AutoConstants.ROTATIONkP, Constants.AutoConstants.ROTATIONkI, Constants.AutoConstants.ROTATIONkD),
      (SwerveModuleState[] states) -> drivetrain.setStates(states),
      events,
      drivetrain);
    
    fullAutoCommand = autoBuilder.fullAuto(trajectories);
    fullAutoCommand.schedule();
  }

  public Command getCommand() {
    return fullAutoCommand;
  }

}