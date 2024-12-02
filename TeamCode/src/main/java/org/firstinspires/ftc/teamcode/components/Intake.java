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

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.custom.DelaySystem;

import java.util.Objects;

public class Intake {
    public enum IntakeState {
        DriverControlled,
        RunningAutomatedIntake,
        Transferring
    }
    static class GatePosition {
        public static final int OPEN = 1;
        public static final double CLOSED = 0;
    }
    public IntakeState state = IntakeState.DriverControlled;
    DelaySystem delaySystem = new DelaySystem();
    DcMotorEx intake, extender;
    ColorSensor leftColorSensor, rightColorSensor;
    DistanceSensor leftDistanceSensor, rightDistanceSensor;
    Servo flipdown, gate;
    RevBlinkinLedDriver display;
    Logger logger;
    //color sensor config
    float minSaturation = 0.4f;
    float minValue = 0.4f;
    float[] hsvValues = new float[3];
    LinearOpMode opMode;
    Gamepad driverController;
    IntakeEvent transferFinished;

    public interface IntakeEvent {
        void fire();
    }

    public Intake(SampleMecanumDrive drive, LinearOpMode opMode, Logger logger, IntakeEvent transferFinished) {
        this.opMode = opMode;
        this.logger = logger;
        this.transferFinished = transferFinished;
        intake = drive.intake;
        leftColorSensor = drive.leftColorSensor;
        rightColorSensor = drive.rightColorSensor;
        leftDistanceSensor = drive.leftDistanceSensor;
        rightDistanceSensor = drive.rightDistanceSensor;
        flipdown = drive.flipdown;
        gate = drive.gate;
        display = drive.display;
        extender = drive.extendo;
        driverController = opMode.gamepad1;
    }

    public boolean IsRunning() {
        return intake.getPower() != 0;
    }

    public void Initialize() {
        //Move intake to flipped up and in
        flipdown.setPosition(0);
        gate.setPosition(GatePosition.CLOSED);
    }

    public void ToggleAutomatedIntake() {
        if (state != IntakeState.RunningAutomatedIntake) {
            state = state == IntakeState.DriverControlled ? IntakeState.RunningAutomatedIntake : IntakeState.DriverControlled;
        }
    }

    float clamp(float num, float min, float max) {
        return Math.max(min, Math.min(num, max));
    }

    public void Update() {
        if (state == IntakeState.RunningAutomatedIntake) {
            if (flipdown.getPosition() != 1) {
                flipdown.setPosition(1);
                intake.setPower(-1);
            }

            //Again, if intake has a sample (not white)
            if (!Objects.equals(GetSampleColor(), "WHITE")) {
                delaySystem.CreateDelay(500, () -> state = IntakeState.DriverControlled);
            }
        }
        else if (state == IntakeState.Transferring) {
            flipdown.setPosition(0);
            gate.setPosition(GatePosition.OPEN);
            intake.setPower(-1);

            if (Objects.equals(GetSampleColor(), "WHITE")) {
                delaySystem.CreateDelay(500, () -> {
                    state = IntakeState.DriverControlled;
                    transferFinished.fire();
                });
            }
        }
        else {
            double power = driverController.right_trigger - driverController.left_trigger;
            if (power != 0) {
                int currentPosition = extender.getCurrentPosition();
                int change = currentPosition - (int)(power * 50);
                extender.setTargetPosition((int)clamp(currentPosition + change, 0, 2000));
            }

            intake.setPower(0);
            flipdown.setPosition(0);
        }

        //Hard stop so intake doesn't flip down while in
        if (extender.getCurrentPosition() > -300) {
            //flipdown.setPosition(0);
        }

        delaySystem.Update();

        opMode.telemetry.addData("Intake State", state);
        opMode.telemetry.addData("Intake Position", extender.getCurrentPosition());
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
