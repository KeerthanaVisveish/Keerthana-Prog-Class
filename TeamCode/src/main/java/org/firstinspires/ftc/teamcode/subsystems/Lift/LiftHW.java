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

    public double getCurrentLiftPosition() {return liftMotor.getCurrentPosition();}

    public void update(double batteryVoltage) {
        double currentPos = getCurrentLiftPosition();

        double feedForwardVoltage = Math.signum(targetPosition - currentPos) * ks;
        double feedBackVoltage = pidController.calculate(currentPos, targetPosition);
        double gravityFeedForwardVoltage = kGLookup.get(currentPos);

        double voltage = feedForwardVoltage + feedBackVoltage + gravityFeedForwardVoltage;
        double power = voltage / batteryVoltage;
        telemetry.addData("Lift Target Power: ", power);
        liftMotor.setPower(power);
    }
    public void setTargetAngle(double desiredPosition) {
        targetPosition = desiredPosition;
    }
}
