package org.firstinspires.ftc.teamcode.components;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.custom.Robot;

public class TransferPlate {
    ColorSensor transferSensor;
    DistanceSensor distanceSensor;
    float minSaturation = 0.4f;
    float minValue = 0.4f;
    float[] hsvValues = new float[3];
    public boolean sampleIsPresent;

    public TransferPlate(Robot robot) {
        transferSensor = robot.drive.transferSensor;
        distanceSensor = robot.drive.transferDistanceSensor;
    }

    public void Update() {
        String sampleColor = "WHITE";
        // Get the color values from the sensor
        int red = transferSensor.red();
        int green = transferSensor.green();
        int blue = transferSensor.blue();

        Color.RGBToHSV(red, green, blue, hsvValues);
        float hue = hsvValues[0];
        float saturation = hsvValues[1];
        float value = hsvValues[2];

        if (saturation >= minSaturation && value >= minValue && distanceSensor.getDistance(DistanceUnit.CM) < 7) {
            //check if color matches any sample colors, or else white
            sampleColor = (hue >= 0 && hue < 65) ? "RED" : (hue >= 65 && hue < 100) ? "YELLOW" : (hue >= 165 && hue < 240) ? "BLUE" : "WHITE";
        }

        sampleIsPresent = !sampleColor.equals("WHITE");
    }
}
