package org.firstinspires.ftc.teamcode.components;

import android.graphics.Color;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Objects;

public class Intake {
    DcMotorEx intake, extender;
    ColorSensor leftColorSensor, rightColorSensor;
    DistanceSensor leftDistanceSensor, rightDistanceSensor;
    Servo flipdown;
    RevBlinkinLedDriver display;
    Logger logger;
    //color sensor config
    float minSaturation = 0.4f;
    float minValue = 0.4f;
    float[] hsvValues = new float[3];
    public boolean runningAutomatedIntake = false;
    LinearOpMode opMode;
    Gamepad assistantController;

    public Intake(SampleMecanumDrive drive, LinearOpMode opMode, Logger logger) {
        this.opMode = opMode;
        this.logger = logger;
        intake = drive.intake;
        leftColorSensor = drive.leftColorSensor;
        rightColorSensor = drive.rightColorSensor;
        leftDistanceSensor = drive.leftDistanceSensor;
        rightDistanceSensor = drive.rightDistanceSensor;
        flipdown = drive.flipdown;
        display = drive.display;
        extender = drive.extendo;
        assistantController = opMode.gamepad2;
    }

    public boolean IsRunning() {
        return intake.getPower() != 0;
    }

    public void Initialize() {
        //Move intake to flipped up and in
        flipdown.setPosition(0);
        intake.setTargetPosition(0);
    }

    public void Update() {
        if (runningAutomatedIntake) {
            if (flipdown.getPosition() != 1) {
                flipdown.setPosition(1);
                intake.setPower(-1);
            }

            //Again, if intake has a sample (not white)
            if (!Objects.equals(GetSampleColor(), "WHITE")) {
                new Thread(() -> {
                    try {
                        Thread.sleep(500);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }
                    intake.setPower(0);
                    flipdown.setPosition(0);
                    runningAutomatedIntake = false;
                }).start();
            }
        }
        else {
            extender.setPower((assistantController.right_trigger - assistantController.left_trigger) * 0.2);
            opMode.telemetry.addData("power", extender.getPower());
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

    String GetSampleColor() {
        String leftSampleColor = QueryColorSensor(leftColorSensor, leftDistanceSensor); //assume robot-forward orientation for these
        String rightSampleColor = QueryColorSensor(rightColorSensor, rightDistanceSensor);
        //If either color sensor returns white (no sample) then set to white, otherwise we know that the intake has a sample
        String sampleColor = Objects.equals(leftSampleColor, "WHITE") || Objects.equals(rightSampleColor, "WHITE") ? "WHITE" : leftSampleColor;
        display.setPattern(BlinkinPattern.valueOf(sampleColor));
        return sampleColor;
    }

    String QueryColorSensor(ColorSensor colorSensor, DistanceSensor distanceSensor) {
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
