package org.firstinspires.ftc.teamcode;

import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm.ArmHW;
import org.firstinspires.ftc.teamcode.subsystems.Lift.LiftHW;
import org.firstinspires.ftc.teamcode.utils.BatteryVoltageFilter;
import org.firstinspires.ftc.teamcode.utils.drivetrain.MecanumDrive;

public class Day3BrainSTEMRobot {
    private MecanumDrive drive;
    public final ArmHW arm;
    public final LiftHW lift;
    private BatteryVoltageFilter batteryVoltageFilter;
    public Day3BrainSTEMRobot(HardwareMap hwMap, Telemetry telemetry, Pose2d initialPose) {
        batteryVoltageFilter = new BatteryVoltageFilter(hwMap);
        arm = new ArmHW(hwMap, telemetry);
        lift = new LiftHW(hwMap, telemetry);
        drive = new MecanumDrive(hwMap, initialPose);
    }

    public void update() {
        double batteryVoltage = batteryVoltageFilter.getVoltage();
        arm.setBatteryVoltage(batteryVoltage);
        lift.setBatteryVoltage(batteryVoltage);

        drive.pinpoint().update();
        arm.update();
        lift.update();
    }
}