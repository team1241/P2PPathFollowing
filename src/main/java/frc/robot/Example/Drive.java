// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Example;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drive extends SubsystemBase {
  /*
   * Example Drive subsystem
   * required methods for using path controller is listed below
   * 
   * should probably also be filled with other stuff for a full robot I think
   */

  // odometry for pose
  private SwerveDriveOdometry odometry;

  protected Drive() {
    odometry = new SwerveDriveOdometry(null, null, null);
  }

  // Requirement: way of controlling robot given ChassisSpeeds object
  public void setChassisVelocityThetaClosedLoop(ChassisSpeeds goalSpeeds) {
    // method implementation ghost is here ooooooOOOOoOOooOOOOOOOOOooo
  }

  // Requirement: way of getting robot pose
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }
}
