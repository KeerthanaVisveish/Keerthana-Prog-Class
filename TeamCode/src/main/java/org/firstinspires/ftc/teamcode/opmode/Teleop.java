package org.firstinspires.ftc.teamcode.opmode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.BrainSTEMRobot;
import org.firstinspires.ftc.teamcode.utils.BatteryVoltageFilter;
import org.firstinspires.ftc.teamcode.utils.ColorSensorWrapper;

@TeleOp (name = "Teleop")
public class Teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
        BatteryVoltageFilter batteryVoltageFilter = new BatteryVoltageFilter(hardwareMap);
        BrainSTEMRobot brainstemRobot = new BrainSTEMRobot(hardwareMap, telemetry, batteryVoltageFilter);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.update();
            batteryVoltageFilter.update();
            brainstemRobot.update();
        }
    }
}
