package frc.robot.TheorySwerveLib;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.interfaces.Gyro;

/*
 * Extend this class to add contructor 
 */

public class TheorySwerve {

    // region ************* Variable Declarations *************

    public enum Modules {
        FRONT_LEFT,
        FRONT_RIGHT,
        BACK_LEFT,
        BACK_RIGHT
    }// Modules

    // Drive Modules
    protected TheoryModule frontLeftModule;
    protected TheoryModule frontRightModule;
    protected TheoryModule backLeftModule;
    protected TheoryModule backRightModule;
    protected ArrayList<TheoryModule> moduleList;

    // Kinematics
    private SwerveDriveKinematics swerveKinematics;
    private ChassisSpeeds cmdSpeeds;

    // Odometry
    private SwerveDriveOdometry odometry;

    // Everytime this runs create an instance of modules
    protected TheorySwerve(double trackWidth, double wheelBase, Rotation2d gyroAngle) {
        swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(trackWidth / 2, wheelBase / 2),
                new Translation2d(trackWidth / 2, -wheelBase / 2),
                new Translation2d(-trackWidth / 2, wheelBase / 2),
                new Translation2d(-trackWidth / 2, -wheelBase / 2));

        odometry = new SwerveDriveOdometry(swerveKinematics, gyroAngle, getModulePositions());

        moduleList = new ArrayList<TheoryModule>(
                Arrays.asList(frontLeftModule, frontRightModule, backLeftModule, backRightModule));
    }

    // endregion

    // region ************* KINEMATICS METHODS *************

    /**
     * 
     * @return commanded speeds of the robot
     */
    public ChassisSpeeds getCmdSpeeds() {
        return cmdSpeeds;
    }

    /**
     * 
     * @return the robots velocity in the local frame
     */
    public ChassisSpeeds getRobotSpeeds() {
        return swerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * 
     * @return the speed of the robot in global / field speeds
     */
    public ChassisSpeeds getFieldSpeeds() {
        ChassisSpeeds robotSpeeds = getRobotSpeeds(); // get the speed of chassis in local frame

        // get vector of robot speeds
        Translation2d robotVelocityVector = new Translation2d(robotSpeeds.vxMetersPerSecond,
                robotSpeeds.vyMetersPerSecond);

        // get the magnitude (speed) of the robot velocity vector
        double robotVelocityVectorMag = robotVelocityVector.getNorm();

        // Local / robot coordinate frame theta angle
        double localTheta;

        localTheta = Math.atan2(robotVelocityVector.getY(), robotVelocityVector.getX());

        // Adding the local theta to the global / field theta
        double fieldThetaOffset = getPose().getRotation().getRadians() + localTheta; // taking the angle from the robot
                                                                                     // pose

        // Find the final (x, y) vector components
        double vyFinal = Math.sin(fieldThetaOffset) * robotVelocityVectorMag;
        double vxFinal = Math.cos(fieldThetaOffset) * robotVelocityVectorMag;

        return new ChassisSpeeds(vxFinal, vyFinal, robotSpeeds.omegaRadiansPerSecond);
    }

    // endregion

    // region ************* ODOMETRY METHODS *************

    /**
     * @return current position of the robot (meters)
     */
    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    /**
     * updates odometry pose with provided gyro angle
     * 
     * @param gyroAngle the angle given by the gryo (degrees)
     */
    public void updateOdometry(double gyroAngle) {
        odometry.update(Rotation2d.fromDegrees(gyroAngle), getModulePositions());
    }

    /**
     * resets robot pose to provided new pose and gyro reading
     * 
     * @param gyroAngle gyro angle (degrees)
     * @param pose      new pose (Pose2d)
     */
    public void resetPose(double gyroAngle, Pose2d pose) {
        odometry.resetPosition(Rotation2d.fromDegrees(gyroAngle), getModulePositions(), pose);
    }

    /**
     * resets robot pose to 0,0 facing +x axis;
     * 
     * @param gyroAngle gyro angle (degrees)
     */
    public void resetPose(double gyroAngle) {
        resetPose(gyroAngle, new Pose2d());
    }

    // endregion

    // region ************* MODULE CONTROL METHODS *************

    /**
     * Sets state of modules given commanded speeds
     * 
     * @param desiredSpeeds desired speed of the robot
     */
    public void setModuleStateInvKin(ChassisSpeeds desiredSpeeds) {
        cmdSpeeds = desiredSpeeds;
        SwerveModuleState[] states = swerveKinematics
                .toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(cmdSpeeds, getPose().getRotation()));
        setModuleStates(states);
    }

    /**
     * 
     * @return the states (velocity and theta) of the modules
     */
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < moduleList.size(); i++) {
            states[i] = moduleList.get(i).getModuleState();
        }
        return states;
    }

    /**
     * @param moduleStates
     */
    public void setModuleStates(SwerveModuleState[] states) {
        SwerveDriveKinematics.desaturateWheelSpeeds(states, ModuleConstants.WHEEL_MAX_SPEED);
        for (int i = 0; i < states.length; i++) {
            moduleList.get(i).setModuleState(states[i]);
        }
    }

    /**
     * 
     * @return the module positions
     */
    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4]; // create an array of modules
        for (int i = 0; i <= moduleList.size(); i++) { // for every module we have
            positions[i] = moduleList.get(i).getModulePosition(); // get position of every module
        }
        return positions; // return array
    }

    /**
     * Sets turn motors to brake
     */
    public void setModulesTurnBrake() {
        for (TheoryModule module : moduleList) {
            module.setTurnBrake();
        }
    }

    /**
     * Sets wheel motors to brake
     */
    public void setModulesWheelBrake() {
        for (TheoryModule module : moduleList) {
            module.setWheelBrake();
        }
    }

    /**
     * Sets turn motors to coast
     */
    public void setModulesTurnCoast() {
        for (TheoryModule module : moduleList) {
            module.setTurnCoast();
        }
    }

    /**
     * Sets wheel motors to coast
     */
    public void setModulesWheelCoast() {
        for (TheoryModule module : moduleList) {
            module.setWheelCoast();
        }
    }

    /**
     * Sets ramp time of modules
     * 
     * @param time (seconds)
     */
    public void setWheelRamp(double time) {
        for (TheoryModule module : moduleList) {
            module.setWheelRamp(time);
        }
    }

    // endregion

}// TheorySwerve
