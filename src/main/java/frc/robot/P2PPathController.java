package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class P2PPathController {
    private PIDController pxController; // posiiton x controller
    private PIDController pyController; // position y controller
    private PIDController thetaController; // heading controller;

    public P2PPathController(P2PTrajectory trajectory, double positionkP, double positionkI, double positionkD,
            double positionTolerance,
            double thetakP, double thetakI, double thetakD, double thetaTolerance, double slewRateVelocity,
            double slewRateOmega) {
        pxController = new PIDController(positionkP, positionkI, positionkD);
        pxController.setTolerance(positionTolerance);

        pyController = new PIDController(positionkP, positionkI, positionkD);
        pyController.setTolerance(positionTolerance);

        thetaController = new PIDController(thetakP, thetakI, thetakD);
        thetaController.setTolerance(thetaTolerance);

    }

    // can I do this one? -andy
    public ChassisSpeeds PoseToPoseControl() {

    }

    public ChassisSpeeds VelocityHeadingControl() {

    }

    private double getDistanceToWaypoint(Pose2d currentPose) {
        Pose2d waypoint = currentTrajectory.getWaypoint();
        double dx = waypoint.getX() - currentPose.getX();
        double dy = waypoint.getY() - currentPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    public boolean isSettled(Pose2d currentPose) {
        return pxController.atSetpoint() && pyController.atSetpoint() && thetaController.atSetpoint();
    }
}