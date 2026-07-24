package org.firstinspires.ftc.teamcode.subsystems.Lift;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class LiftHW {
    private final DcMotorEx liftMotor;
    private final Telemetry telemetry;
    private final PIDController pidController;
    public static double kP = 0, kI = 0, kD = 0;
    public static double ks = 0;
    public static double dampeningBand = 1;
    private double targetPosition;
    private InterpLUT kGLookup = new InterpLUT();
    private double batteryVoltage = 13;

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

    public double getCurrentLiftPosition() {return liftMotor.getCurrentPosition();}

    public void update() {
        double currentPos = getCurrentLiftPosition();
        double error = targetPosition - currentPos;

        double scale = Math.min(1, Math.abs(error) / dampeningBand);

        double feedForwardVoltage = Math.signum(error) * ks * scale;
        double feedBackVoltage = pidController.calculate(currentPos, targetPosition) * scale;
        double gravityFeedForwardVoltage = kGLookup.get(currentPos);

        double voltage = feedForwardVoltage + feedBackVoltage + gravityFeedForwardVoltage;
        double power = voltage / batteryVoltage;
        telemetry.addData("Lift Target Power: ", power);
        liftMotor.setPower(power);
    }

    public void setBatteryVoltage(double voltage) {batteryVoltage = voltage;}

    public void setTargetPosition(double desiredPosition) {
        targetPosition = desiredPosition;
    }
}