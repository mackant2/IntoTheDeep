package org.firstinspires.ftc.teamcode.drive.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.custom.PressEventSystem;


@TeleOp (group = "tuning", name = "[TUNING] FourBar")
public class FourBar extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        PressEventSystem pressEventSystem = new PressEventSystem(telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo leftFourBar, rightFourBar, wrist, claw;
        leftFourBar = drive.leftFourBar;
        rightFourBar = drive.rightFourBar;
        claw = drive.claw;
        wrist = drive.wrist;

        waitForStart();

        claw.setPosition(0.8);

        pressEventSystem.AddListener(gamepad1, "x", () -> {leftFourBar.setPosition(0);rightFourBar.setPosition(0);});
        pressEventSystem.AddListener(gamepad1, "a", () -> {leftFourBar.setPosition(0.5);rightFourBar.setPosition(0.5);});
        pressEventSystem.AddListener(gamepad1, "b", () -> {leftFourBar.setPosition(1);rightFourBar.setPosition(1);});
        pressEventSystem.AddListener(gamepad1, "y", () -> claw.setPosition(claw.getPosition() == 0.8 ? 0.1 : 0.8));

        while (!isStopRequested()) {
            pressEventSystem.Update();

            wrist.setPosition(gamepad1.right_trigger);
            telemetry.addData("Four Bar Position", leftFourBar.getPosition());
            telemetry.addData("Claw Position", claw.getPosition());
            telemetry.addData("Wrist Position", wrist.getPosition());
            telemetry.update();
        }
    }
}