package org.firstinspires.ftc.teamcode.subsystems.Arm;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ArmHW {
    private final DcMotorEx armMotor;
    private final Telemetry telemetry;
    private final PIDController pidController;
    public static double kP = 0, kI = 0, kD = 0;
    public static double kG = 0, ks = 0;
    private final int radsPerTick = 0; // get from specs, don't tune
    private double targetAngle = 0;

    public ArmHW(HardwareMap hwMap, Telemetry telemetry) {
        armMotor = hwMap.get(DcMotorEx.class, "arm");
        pidController = new PIDController(kP, kI, kD);
        this.telemetry = telemetry;
    }

    public double getCurrentArmPosition() {return armMotor.getCurrentPosition();}

    public void update(double batteryVoltage) {
        pidController.setPID(kP, kI, kD);
        double currentAngle = getCurrentArmPosition() * radsPerTick;

        double feedForwardVoltage = Math.signum(targetAngle - currentAngle) * ks;
        double feedBackVoltage = pidController.calculate(currentAngle, targetAngle);
        double gravityFeedForwardVoltage = kG * Math.cos(currentAngle);

        double voltage = feedForwardVoltage + feedBackVoltage + gravityFeedForwardVoltage;
        double power = voltage / batteryVoltage;

        power = Range.clip(power, -1, 1);
        telemetry.addData("Arm Target Power: ", power);
        armMotor.setPower(power);
    }

    public void setTargetAngle(double desiredAngle) { // angle in rads
        targetAngle = desiredAngle;
    }
}