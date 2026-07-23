package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Arm.ArmHW;
import org.firstinspires.ftc.teamcode.subsystems.Lift.LiftHW;
import org.firstinspires.ftc.teamcode.utils.BatteryVoltageFilter;

public class BrainSTEMRobot {
    private ArmHW arm;
    private LiftHW lift;
    private BatteryVoltageFilter batteryVoltageFilter;
    public BrainSTEMRobot(HardwareMap hwMap, Telemetry telemetry, BatteryVoltageFilter batteryVoltageFilter) {
        this.batteryVoltageFilter = new BatteryVoltageFilter(hwMap);
        arm = new ArmHW(hwMap, telemetry);
        lift = new LiftHW(hwMap, telemetry);
    }

    public void update() {
        double batteryVoltage = batteryVoltageFilter.getVoltage();
        arm.update(batteryVoltage);
        lift.update(batteryVoltage);
    }
}
