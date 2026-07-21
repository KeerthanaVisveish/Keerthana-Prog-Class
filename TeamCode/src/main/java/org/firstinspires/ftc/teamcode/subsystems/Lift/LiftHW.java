package org.firstinspires.ftc.teamcode.subsystems.Lift;

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftHW {
    private final DcMotorEx liftMotor;
    private final Telemetry telemetry;
    private final PIDController pidController;
    private final double kP = 0, kI = 0, kD = 0;
    private final double kG = 0, ks = 0;
    private double targetPosition;
    private InterpLUT kGLookup = new InterpLUT();

    public LiftHW(HardwareMap hwMap, Telemetry telemetry) {
        liftMotor = hwMap.get(DcMotorEx.class, "lift");
        pidController = new PIDController(kP, kI, kD);
        this.telemetry = telemetry;
        populateKGLookup();
    }

    private void populateKGLookup(){
        kGLookup.add(0, 0);
        kGLookup.add(50, 0);
        kGLookup.add(100, 0);
        kGLookup.add(150, 0);
    }

    private double getCurrentLiftPosition() {return liftMotor.getCurrentPosition();}

    private void update() {
        double currentPos = getCurrentLiftPosition();
        double feedForward = Math.signum(targetPosition - currentPos) * ks;
        double feedBack = pidController.calculate(currentPos, targetPosition);
        double gravityFeedForward = kGLookup.get(currentPos);
        double power = feedForward + feedBack + gravityFeedForward;
        telemetry.addData("Lift Target Power: ", power);
        liftMotor.setPower(power);
    }
    private void setTargetAngle(double desiredPosition) {
        targetPosition = desiredPosition;
    }
}
