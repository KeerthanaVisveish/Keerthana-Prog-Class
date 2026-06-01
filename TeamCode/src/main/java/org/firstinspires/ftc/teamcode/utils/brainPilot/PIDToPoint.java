package org.firstinspires.ftc.teamcode.utils.brainPilot;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.Range;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Vector2d;

@Config
public class PIDToPoint extends CommandBase {
    // tuning params
    public static double defaultSpeedKp = 1;
    public static double defaultHeadingKp = 0.05;

    // tolerances
    public static double positionTolerance = 1.0;
    public static double headingToleranceDeg = 3.0;

    // power limits
    public static double maxDrivePower = 1.0;
    public static double minDrivePower = 0.08;
    public static double maxTurnPower = 1.0;


    private final MecanumDrive drivetrain;
    private final Pose2d targetPose;

    // proportional gains
    private final double speedKp;
    private final double headingKp;

    private boolean isFinished = false;


    public PIDToPoint(MecanumDrive drivetrain, Pose2d targetPose) {
        this(drivetrain, targetPose, defaultSpeedKp, defaultHeadingKp);
    }
    public PIDToPoint(MecanumDrive drivetrain, Pose2d targetPose, double speedKp, double headingKp) {
        this.drivetrain = drivetrain;
        this.targetPose = targetPose;

        this.speedKp = speedKp;
        this.headingKp = headingKp;

        addRequirements();
    }

    @Override
    public void execute() {
        Pose2d robotPose = drivetrain.pinpoint().getPose();

        // position error
        Vector2d robotToTarget = targetPose.position.minus(robotPose.position);
        double distanceError = robotToTarget.norm();
        Vector2d robotToTargetDir = robotToTarget.div(distanceError);

        // proportional drive power
        double drivePower = distanceError * speedKp;
        drivePower = distanceError > positionTolerance ? Range.clip(drivePower, minDrivePower, maxDrivePower) : 0;
        Vector2d driveVector = robotToTargetDir.times(drivePower);

        // heading proportional control
        double headingErrorRad = targetPose.heading.minus(robotPose.heading);
        double turnPower = headingErrorRad * headingKp;
        turnPower = Range.clip(turnPower, -maxTurnPower, maxTurnPower);

        drivetrain.setDrivePowers(new PoseVelocity2d(driveVector, turnPower));

        isFinished = distanceError < positionTolerance && Math.abs(headingErrorRad) < Math.toRadians(headingToleranceDeg);
    }

    @Override
    public boolean isFinished() {
        return isFinished;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
    }
}