package org.firstinspires.ftc.teamcode.subsystems.Lift;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;

public class LiftHW {
    private DcMotorEx liftMotor;
    private PIDController pidController;
    private double kP = 0, kI = 0, kD = 0;
    private double kG = 0, ks = 0;

    public LiftHW(HardwareMap hwMap, Telemetry telemetry) {
        liftMotor = hwMap.get(DcMotorEx.class, "lift");
        pidController = new PIDController(kP, kI, kD);
    }

    private double getCurrentLiftPosition() {return liftMotor.getCurrentPosition();}

    private void setLiftPosition(double targetPosition) {
        double currentPos = getCurrentLiftPosition();
        double feedForward = Math.signum(targetPosition - currentPos) * ks;
        double feedBack = pidController.calculate(currentPos, targetPosition);
        double power = feedForward + feedBack + kG;
        liftMotor.setPower(power);
    }
}
