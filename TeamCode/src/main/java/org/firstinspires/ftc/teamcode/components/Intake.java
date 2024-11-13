package org.firstinspires.ftc.teamcode.components;
import android.graphics.Color;

import com.acmerobotics.roadrunner.trajectory.MarkerCallback;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicBoolean;

public class Intake {
    DcMotor intake;
    ColorSensor leftColorSensor;
    ColorSensor rightColorSensor;
    DistanceSensor leftDistanceSensor;
    DistanceSensor rightDistanceSensor;
    Logger logger;
    //color sensor config
    float minSaturation = 0.4f;
    float minValue = 0.4f;
    float[] hsvValues = new float[3];
    AtomicBoolean stopping = new AtomicBoolean(false);
    double power = 0;

    public Intake(SampleMecanumDrive drive, Logger _logger) {
        logger = _logger;
        intake = drive.Intake;
        leftColorSensor = drive.leftColorSensor;
        rightColorSensor = drive.rightColorSensor;
        leftDistanceSensor = drive.leftDistanceSensor;
        rightDistanceSensor = drive.rightDistanceSensor;
    }

    public boolean IsRunning() {
        return intake.getPower() != 0;
    }

    public void Update() {
        boolean leftHasSample = HasSample(leftColorSensor, leftDistanceSensor); //assume robot-forward orientation for these
        boolean rightHasSample = HasSample(rightColorSensor, rightDistanceSensor);

        if (leftHasSample && rightHasSample && !stopping.get()) {
            stopping.set(true);
            new Thread(() -> {
                try {
                    Thread.sleep(500);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
                intake.setPower(-0.01);
                stopping.set(false);
            }).start();
        }
        else if (intake.getPower() != power && (leftDistanceSensor.getDistance(DistanceUnit.CM) > 7 || rightDistanceSensor.getDistance(DistanceUnit.CM) > 7)) {
            intake.setPower(power);
        }
    }

    public void SetRunning(boolean enabled) {
        power = enabled ? -1 : 0;
    }

    public MarkerCallback Outtake = () -> {
        logger.Log("samplestore started outtake");
    };

    public MarkerCallback Intake = () -> {
        logger.Log("samplestore started intake");
    };

    boolean HasSample(ColorSensor colorSensor, DistanceSensor distanceSensor) {
        boolean hasSample = false;
        // Get the color values from the sensor
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();

        Color.RGBToHSV(red, green, blue, hsvValues);
        float hue = hsvValues[0];
        float saturation = hsvValues[1];
        float value = hsvValues[2];

        if (saturation >= minSaturation && value >= minValue && distanceSensor.getDistance(DistanceUnit.CM) < 7) {
            //check if color matches any sample colors
            hasSample = (hue >= 0 && hue < 65) || (hue >= 65 && hue < 100) || (hue >= 165 && hue < 240);
        }

        return hasSample;
    }
}
