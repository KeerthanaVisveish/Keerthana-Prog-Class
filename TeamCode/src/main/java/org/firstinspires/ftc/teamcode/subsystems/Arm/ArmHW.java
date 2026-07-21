package org.firstinspires.ftc.teamcode.subsystems.Arm;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ArmHW {
    private DcMotorEx armMotor;
    private PIDController pidController;
    private double kP = 0, kI = 0, kD = 0;
    private double kG = 0, ks = 0;
    private int radsPerTick = 0;

    public ArmHW(HardwareMap hwMap, Telemetry telemetry) {
        armMotor = hwMap.get(DcMotorEx.class, "arm");
        pidController = new PIDController(kP, kI, kD);
    }

    private double getCurrentArmPosition() {return armMotor.getCurrentPosition();}

    private void setArmPosition(double targetAngle) { // angle in rads
        double currentAngle = getCurrentArmPosition() * radsPerTick;
        double feedForward = Math.signum(targetAngle - currentAngle) * ks;
        double feedBack = pidController.calculate(currentAngle, targetAngle);
        double gravityFeedForward = kG * Math.cos(currentAngle);
        double power = feedForward + feedBack + gravityFeedForward;
        armMotor.setPower(power);
    }
}