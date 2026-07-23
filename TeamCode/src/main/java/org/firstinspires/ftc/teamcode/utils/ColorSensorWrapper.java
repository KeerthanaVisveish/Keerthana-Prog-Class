package org.firstinspires.ftc.teamcode.utils;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class ColorSensorWrapper {

    public static double minR = 0, maxR = 0;
    public static double minG = 0, maxG = 0;
    public static double minB = 0, maxB = 0;

    // cached color sensor values
    private double red, green, blue;
    private final ColorSensor colorSensor;
    private final Telemetry telemetry;

    public ColorSensorWrapper(HardwareMap hwMap, Telemetry telemetry) {
        this.telemetry = telemetry;
        colorSensor = hwMap.get(ColorSensor.class, "colorSensor");
    }

    public void update() {
        red = colorSensor.red();
        green = colorSensor.green();
        blue = colorSensor.blue();

        double brightness = red + green + blue;

        red = red / brightness;
        green = green / brightness;
        blue = blue / brightness;

        telemetry.addLine("-- COLOR SENSOR --");
        telemetry.addData("Red: ", red);
        telemetry.addData("Green: ", green);
        telemetry.addData("Blue: ", blue);
        telemetry.addData("Sees Ball: ", seesColorInRange());
    }

    public boolean seesColorInRange() {
        boolean redValid = red <= maxR && red >= minR;
        boolean greenValid = green <= maxG && green >= minG;
        boolean blueValid = blue <= maxB && blue >= minB;

        return redValid && greenValid & blueValid;
    }
}
