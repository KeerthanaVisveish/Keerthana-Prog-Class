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
    public static double deadbandRadians = 0;
    private final int radsPerTick = 0; // get from specs, don't tune
    private double targetAngle = 0;
    private double batteryVoltage = 13;

    public ArmHW(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        armMotor = hwMap.get(DcMotorEx.class, "arm");
        pidController = new PIDController(kP, kI, kD);
    }

    public void update() {
        pidController.setPID(kP, kI, kD);
        double currentAngle = getCurrentArmPosition() * radsPerTick;
        double error = targetAngle - currentAngle;

        double feedForwardVoltage = Math.signum(error) * ks;
        double feedBackVoltage = pidController.calculate(currentAngle, targetAngle);

        if (Math.abs(error) < deadbandRadians) {
            feedForwardVoltage = 0;
            feedBackVoltage = 0;
        }

        double gravityFeedForwardVoltage = kG * Math.cos(currentAngle);

        double voltage = feedForwardVoltage + feedBackVoltage + gravityFeedForwardVoltage;
        double power = voltage / batteryVoltage;

        power = Range.clip(power, -1, 1);
        telemetry.addData("Arm Target Power: ", power);
        armMotor.setPower(power);
    }

    public double getCurrentArmPosition() {return armMotor.getCurrentPosition();}

    public void setBatteryVoltage(double voltage) {batteryVoltage = voltage;}

    public void setTargetAngle(double desiredAngle) { // angle in rads
        targetAngle = desiredAngle;
    }
}