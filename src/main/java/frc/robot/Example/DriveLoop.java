package frc.robot.Example;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import frc.robot.P2PPathController;
import frc.robot.P2PTrajectory;

public class DriveLoop extends Drive {

    public enum DriveStates {
        PATH_FOLLOWING,
        DISABLED
    }

    private static DriveLoop mInstance;
    private P2PTrajectory currentTrajectory;
    private DriveStates currentState;
    private P2PPathController pathController;

    public static DriveLoop getInstance() {
        if (mInstance == null) {
            mInstance = new DriveLoop();
        }
        return mInstance;
    }

    private DriveLoop() {
        currentTrajectory = TrajectoryConstants.TO_SECOND_GAMEPIECE_BUMP_RED;

        // instantiate pathfollower class
        // im sorry who ever is filling this out
        pathController = new P2PPathController(currentTrajectory, 0, 0, 0, 0,
                0, 0, 0, 0, 0, 0);
    }

    @Override
    public void periodic() {
        switch (currentState) {

            /*
             * Call getGoalSpeeds to get required ChassisSpeeds object to follow path
             * feed result into control method
             */
            case PATH_FOLLOWING:
                super.setChassisVelocityThetaClosedLoop(pathController.getGoalSpeeds(super.getPose(), 5));
                break;

            default:
            case DISABLED:
                // stop power, other stuff, etc, things...
                break;
        }
    }

    public void setCurrentTrajectory(P2PTrajectory newTrajectory) {
        this.currentTrajectory = newTrajectory;
        pathController.setTrajectory(currentTrajectory);
    }

    public void setState(DriveStates newState) {
        this.currentState = newState;
    }

    public void resetPathController() {
        pathController.reset();

    }

    public boolean isPathControllerSettled() {
        return pathController.isSettled();
    }

}
