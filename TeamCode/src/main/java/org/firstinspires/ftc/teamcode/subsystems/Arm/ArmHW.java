package org.firstinspires.ftc.teamcode.subsystems.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmHW {
    private final DcMotorEx armMotor;
    private final Telemetry telemetry;
    private final PIDController pidController;
    public static double kP = 0, kI = 0, kD = 0;
    public static double kG = 0, ks = 0;
    private final int radsPerTick = 0;
    private double targetAngle = 0;

    public ArmHW(HardwareMap hwMap, Telemetry telemetry) {
        armMotor = hwMap.get(DcMotorEx.class, "arm");
        pidController = new PIDController(kP, kI, kD);
        this.telemetry = telemetry;
    }

    public double getCurrentArmPosition() {return armMotor.getCurrentPosition();}

    private void update() {
        double currentAngle = getCurrentArmPosition() * radsPerTick;
        double feedForward = Math.signum(targetAngle - currentAngle) * ks;
        double feedBack = pidController.calculate(currentAngle, targetAngle);
        double gravityFeedForward = kG * Math.cos(currentAngle);
        double power = feedForward + feedBack + gravityFeedForward;
        telemetry.addData("Arm Target Power: ", power);
        armMotor.setPower(power);
    }

    public void setTargetAngle(double desiredAngle) { // angle in rads
        targetAngle = desiredAngle;
    }
}