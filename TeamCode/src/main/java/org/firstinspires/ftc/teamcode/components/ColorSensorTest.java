package org.firstinspires.ftc.teamcode.components;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;

@TeleOp(name = "ColorSensorTest", group = "Sensor")
public class ColorSensorTest extends LinearOpMode {

    private ColorSensor colorSensor;
    private DistanceSensor sensorDistance;
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;


    @Override
    public void runOpMode() {
        // Initialize the color sensor
        colorSensor = hardwareMap.get(ColorSensor.class, "color_sensor");
        sensorDistance = hardwareMap.get(DistanceSensor.class, "color_sensor");

        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
        blinkinLedDriver.setPattern(pattern);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        while (opModeIsActive()) {

            // Get the color values from the sensor
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            // Convert RGB to HSV
            float[] hsvValues = new float[3];
            Color.RGBToHSV(red, green, blue, hsvValues);
            float hue = hsvValues[0];
            float saturation = hsvValues[1];
            float value = hsvValues[2];

            float minSaturation = 0.4f; // Adjust as necessary
            float minValue = 0.4f; // Adjust as necessary

            // Check for colors based on hue
            if (saturation >= minSaturation && value >= minValue) {
                if (hue >= 0 && hue < 65) {
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
                    telemetry.addData("Color", "Red");
                } else if (hue >= 65 && hue < 100) {
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.YELLOW);
                    telemetry.addData("Color", "Yellow");
                } else if (hue >= 165 && hue < 240) {
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
                    telemetry.addData("Color", "Blue");
                } else {
                    blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                    telemetry.addData("Color", "Unknown");
                }
            } else {
                blinkinLedDriver.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
                telemetry.addData("Color", "Unknown");
            }


            telemetry.addData("Current hue", hue);
            telemetry.addData("Current satruration", saturation);
            telemetry.addData("Current value", value);
            telemetry.addData("Distance (cm)",
                    String.format(Locale.US, "%.02f", sensorDistance.getDistance(DistanceUnit.CM)));

            /*

            // Get the color values from the sensor
            int red = colorSensor.red();
            int green = colorSensor.green();
            int blue = colorSensor.blue();

            // Calculate the ratios
            double total = red + green + blue;
            double redRatio = red / total;
            double greenRatio = green / total;
            double blueRatio = blue / total;


            // Check for colors
            if (redRatio > 0.4 && greenRatio < 0.3 && blueRatio < 0.3) {
                telemetry.addData("Color", "Red");
            } else if (redRatio > 0.3 && greenRatio > 0.3 && blueRatio < 0.3) {
                telemetry.addData("Color", "Yellow");
            } else if (blueRatio > 0.4 && redRatio < 0.3 && greenRatio < 0.3) {
                telemetry.addData("Color", "Blue");
            } else {
                telemetry.addData("Color", "Unknown");
            }

            telemetry.addData("Red", red);
            telemetry.addData("Blue", blue);
            telemetry.addData("Green", green);
            */


            telemetry.update();
        }
    }
}
