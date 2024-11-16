package org.firstinspires.ftc.teamcode.components;
import android.graphics.Color;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Objects;
import java.util.concurrent.atomic.AtomicBoolean;

public class Intake {
    DcMotor intake;
    ColorSensor leftColorSensor;
    ColorSensor rightColorSensor;
    DistanceSensor leftDistanceSensor;
    DistanceSensor rightDistanceSensor;
    Servo flipdown;
    RevBlinkinLedDriver display;
    Logger logger;
    //color sensor config
    float minSaturation = 0.4f;
    float minValue = 0.4f;
    float[] hsvValues = new float[3];
    AtomicBoolean stopping = new AtomicBoolean(false);
    boolean flippedDown = false;
    LinearOpMode opMode;

    public Intake(SampleMecanumDrive drive, LinearOpMode opMode, Logger logger) {
        this.opMode = opMode;
        this.logger = logger;
        intake = drive.Intake;
        leftColorSensor = drive.leftColorSensor;
        rightColorSensor = drive.rightColorSensor;
        leftDistanceSensor = drive.leftDistanceSensor;
        rightDistanceSensor = drive.rightDistanceSensor;
        flipdown = drive.flipdown;
        display = drive.display;
    }

    public void Run() {
        if (!flippedDown) {
            flippedDown = true;
            flipdown.setPosition(1);
            intake.setPower(-1);
        }
    }

    public boolean IsRunning() {
        return intake.getPower() == -1;
    }

    public void Update() {
        String leftSampleColor = GetSampleColor(leftColorSensor, leftDistanceSensor); //assume robot-forward orientation for these
        String rightSampleColor = GetSampleColor(rightColorSensor, rightDistanceSensor);

        //If either color sensor returns white (no sample) then set to white, otherwise we know that the intake has a sample
        String pattern = Objects.equals(leftSampleColor, "WHITE") || Objects.equals(rightSampleColor, "WHITE") ? "WHITE" : leftSampleColor;

        display.setPattern(BlinkinPattern.valueOf(pattern));

        //Again, if intake has a sample (not white)
        if (!Objects.equals(pattern, "WHITE")) {
            if (!stopping.get()) {
                stopping.set(true);
                new Thread(() -> {
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    intake.setPower(0);
                    flippedDown = false;
                    flipdown.setPosition(0);
                    stopping.set(false);
                }).start();
            }
        }
    }

    public void SetRunning(boolean enabled) {
        //intake.setPower(enabled ? -1 : 0);
    }

    public MarkerCallback Outtake = () -> {
        logger.Log("samplestore started outtake");
    };

    public MarkerCallback Intake = () -> {
        logger.Log("samplestore started intake");
    };

    String GetSampleColor(ColorSensor colorSensor, DistanceSensor distanceSensor) {
        String sampleColor = "WHITE";
        // Get the color values from the sensor
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        Color.RGBToHSV(red, green, blue, hsvValues);
        float hue = hsvValues[0];
        float saturation = hsvValues[1];
        float value = hsvValues[2];

        if (saturation >= minSaturation && value >= minValue && distanceSensor.getDistance(DistanceUnit.CM) < 7) {
            //check if color matches any sample colors, or else white
            sampleColor = (hue >= 0 && hue < 65) ? "RED" : (hue >= 65 && hue < 100) ? "YELLOW" : (hue >= 165 && hue < 240) ? "BLUE" : "WHITE";
        }

        return sampleColor;
    }
}
