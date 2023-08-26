package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class P2PPathController {
    private PIDController pxController; // posiiton x controller
    private PIDController pyController; // position y controller
    private PIDController thetaController; // heading controller;
    private SlewRateLimiter vxSlewLimiter;
    private SlewRateLimiter vySlewLimiter;
    private SlewRateLimiter omegaSlewLimiter;
    public P2PTrajectory currentTrajectory;

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

        vxSlewLimiter = new SlewRateLimiter(0.5);
        vySlewLimiter = new SlewRateLimiter(0.5);
        omegaSlewLimiter = new SlewRateLimiter(0.5);



    }

    // can I do this one? -andy
    public ChassisSpeeds PoseToPoseControl() {

    }

    public ChassisSpeeds VelocityHeadingControl() {

    }

    private double getDistanceToWaypoint(Pose2d currentPose) {
        Pose2d waypoint = currentTrajectory.getCurrentWayPoint().getPose();
        double dx = waypoint.getX() - currentPose.getX();
        double dy = waypoint.getY() - currentPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    private double getAngleToWaypoint(Pose2d currentPose){
        double targetAngle = currentTrajectory.getCurrentWayPoint().getRotation().getDegrees();
        return 
    }


    public boolean isSettled(Pose2d currentPose) {
        return pxController.atSetpoint() && pyController.atSetpoint() && thetaController.atSetpoint();
    }

    public ChassisSpeeds getGoalSpeeds(P2PTrajectory currentPose, double velocitySetpoint) {
        if(currentPose.isEndPoint()){
            
        }
        else{
            if(currentPose)

        }
    }

    public void setTrajectory(P2PTrajectory newTrajectory) {
        this.currentTrajectory = newTrajectory;
    }

    public void setPositionPID(double kP, double kI, double kD) {

    }

    private boolean inSetpointRadius(Pose2d currentPose){
        
    }

}