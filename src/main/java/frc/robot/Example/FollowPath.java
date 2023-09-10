// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Example;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.P2PTrajectory;
import frc.robot.Example.DriveLoop.DriveStates;

public class FollowPath extends CommandBase {
  private DriveLoop mDriveLoop;
  private P2PTrajectory mTrajectory;

  /** Creates a new FollowPath. */
  public FollowPath(P2PTrajectory trajectory) {
    this.mDriveLoop = DriveLoop.getInstance();
    this.mTrajectory = trajectory;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set trajectory of the path controller to the passed in trajectory
    mDriveLoop.setCurrentTrajectory(mTrajectory);

    // reset controller PIDs, SlewRateLimiters, Trajectory instance
    mDriveLoop.resetPathController();

    mDriveLoop.setState(DriveStates.PATH_FOLLOWING);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // stop motors and reset stuff here
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // check if PIDs have reached setpoints
    return mDriveLoop.isPathControllerSettled();
  }
}
