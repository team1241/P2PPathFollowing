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

        vxSlewLimiter = new SlewRateLimiter(slewRateVelocity);
        vySlewLimiter = new SlewRateLimiter(slewRateVelocity);
        omegaSlewLimiter = new SlewRateLimiter(slewRateOmega);

    }

    // *************** Calculation Methods ***************
    public ChassisSpeeds getGoalSpeeds(P2PTrajectory currentPose, double velocitySetpoint) {
        if(currentPose.isCurrentEndPoint()){
            
        }
        else{
            if(currentPose)

        }
    }

    public ChassisSpeeds PoseToPoseControl(Pose2d currentPose) {
        Pose2d targetPose = currentTrajectory.getCurrentWaypoint().getPose();

        double vx = vxSlewLimiter.calculate(pxController.calculate(currentPose.getX(), targetPose.getX()));
        double vy = vySlewLimiter.calculate(pyController.calculate(currentPose.getY(), targetPose.getY()));
        double omega = omegaSlewLimiter.calculate(thetaController.calculate(currentPose.getRotation().getRadians(),
                targetPose.getRotation().getRadians()));

        return new ChassisSpeeds(vx, vy, omega);
    }

    public ChassisSpeeds VelocityHeadingControl(P2PTrajectory currentPose, double velocitySetpoint) {

    }

    // *************** Util Methods ***************

    private double getDistanceToWaypoint(Pose2d currentPose) {
        Pose2d waypoint = currentTrajectory.getCurrentWaypoint().getPose();
        double dx = waypoint.getX() - currentPose.getX();
        double dy = waypoint.getY() - currentPose.getY();
        return Math.sqrt(dx * dx + dy * dy);
    }

    private double getAngleToWaypoint(Pose2d currentPose){
        double targetAngle = currentTrajectory.getCurrentWaypoint().getPose().getRotation().getDegrees();
        return 
    }

    private boolean inSetpointRadius(Pose2d currentPose) {

    }

    // *************** Other Methods ***************

    public boolean isSettled(Pose2d currentPose) {
        return pxController.atSetpoint() && pyController.atSetpoint() && thetaController.atSetpoint()
                && currentTrajectory.isCurrentEndPoint();
    }

    public void setTrajectory(P2PTrajectory newTrajectory) {
        this.currentTrajectory = newTrajectory;
    }

    public void setPositionPID(double kP, double kI, double kD) {

    }

    public void setThetaPID(double kP, double kI, double kD) {
        thetaController.setPID(kP, kI, kD);
    }

    public void setVelocitySlewRate(double rate) {
        vxSlewLimiter = new SlewRateLimiter(rate);
        vySlewLimiter = new SlewRateLimiter(rate);
    }

    public void setOmegaSlewRate(double rate) {
        vxSlewLimiter = new SlewRateLimiter(rate);
        vySlewLimiter = new SlewRateLimiter(rate);
    }

}