package org.firstinspires.ftc.teamcode.components;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachineBuilder;
import com.sfdev.assembly.state.StateMachine;

import org.apache.commons.math3.analysis.integration.IterativeLegendreGaussIntegrator;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.custom.DelaySystem;
import org.firstinspires.ftc.teamcode.util.custom.Robot;

import java.util.Objects;

public class Intake {
    public enum IntakeState {
        DriverControlled,
        RunningAutomatedIntake,
        Transferring,
        Rejecting
    }
    static class GatePosition {
        public static final int OPEN = 1;
        public static final double CLOSED = 0;
    }
    static class ExtenderPosition {
        public static final int IN = 10;
        public static final int OUT = 2000;
    }
    public IntakeState state = IntakeState.DriverControlled;
    DelaySystem delaySystem = new DelaySystem();
    DcMotorEx intake, extender;
    ColorSensor leftColorSensor, rightColorSensor;
    DistanceSensor leftDistanceSensor, rightDistanceSensor;
    Servo flipdown, gate;
    RevBlinkinLedDriver display;
    Robot robot;
    //color sensor config
    float minSaturation = 0.4f;
    float minValue = 0.4f;
    float[] hsvValues = new float[3];
    Gamepad driverController;
    StateMachine stateMachine;

    public Intake(SampleMecanumDrive drive, Robot robot) {
        this.robot = robot;
        intake = drive.intake;
        leftColorSensor = drive.leftColorSensor;
        rightColorSensor = drive.rightColorSensor;
        leftDistanceSensor = drive.leftDistanceSensor;
        rightDistanceSensor = drive.rightDistanceSensor;
        flipdown = drive.flipdown;
        gate = drive.gate;
        display = drive.display;
        extender = drive.extendo;
        driverController = robot.opMode.gamepad1;

        stateMachine = new StateMachineBuilder()
                .state(IntakeState.DriverControlled)
                .transition(() -> driverController.right_bumper)
                .build();
    }

    public void Initialize() {
        //Move intake to flipped up and in
        flipdown.setPosition(0);
        delaySystem.CreateDelay(500, () -> extender.setTargetPosition(ExtenderPosition.IN));
        gate.setPosition(GatePosition.CLOSED);
        extender.setTargetPosition(ExtenderPosition.IN);
    }

    public void ToggleAutomatedIntake() {
        if (state != IntakeState.RunningAutomatedIntake) {
            state = state == IntakeState.DriverControlled ? IntakeState.RunningAutomatedIntake : IntakeState.DriverControlled;
        }
    }

    public void ToggleRejection() {
        state = state == IntakeState.Rejecting ? IntakeState.DriverControlled : IntakeState.Rejecting;
    }

    float clamp(float num, float min, float max) {
        return Math.max(min, Math.min(num, max));
    }

    public void Update() {
        if (state == IntakeState.RunningAutomatedIntake) {
            gate.setPosition(GatePosition.CLOSED);
            if (flipdown.getPosition() != 1 || intake.getPower() != -1) {
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

            if (robot.transferPlate.sampleIsPresent) {
                state = IntakeState.DriverControlled;
            }
        }
        else if (state == IntakeState.DriverControlled) {
            double power = driverController.right_trigger - driverController.left_trigger;
            if (power != 0) {
                int currentPosition = extender.getCurrentPosition();
                int target = currentPosition + (int)Math.round(power * 500);
                extender.setTargetPosition((int)clamp(target, ExtenderPosition.IN, ExtenderPosition.OUT));
            }

            gate.setPosition(GatePosition.CLOSED);
            intake.setPower(0);
            flipdown.setPosition(0);
        }
        else if (state == IntakeState.Rejecting) {
            gate.setPosition(GatePosition.OPEN);
            intake.setPower(1);
            flipdown.setPosition(1);
        }

        delaySystem.Update();

        robot.opMode.telemetry.addData("Intake State", state);
        robot.opMode.telemetry.addData("Extender Voltage (MILLIAMPS)", extender.getCurrent(CurrentUnit.MILLIAMPS));
        robot.opMode.telemetry.addData("Extender Velocity", extender.getVelocity());
        robot.opMode.telemetry.addData("Extender Position", extender.getCurrentPosition());
        robot.opMode.telemetry.addData("Extender Target Position", extender.getTargetPosition());
    }

    String GetSampleColor() {
        String leftSampleColor = QueryColorSensor(leftColorSensor, leftDistanceSensor); //assume robot-forward orientation for these
        String rightSampleColor = QueryColorSensor(rightColorSensor, rightDistanceSensor);
        //If either color sensor returns white (no sample) then set to white, otherwise we know that the intake has a sample
        String sampleColor = /*Objects.equals(leftSampleColor, "WHITE")*/ false || Objects.equals(rightSampleColor, "WHITE") ? "WHITE" : rightSampleColor;
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
