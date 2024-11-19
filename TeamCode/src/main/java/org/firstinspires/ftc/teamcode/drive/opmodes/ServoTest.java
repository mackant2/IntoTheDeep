package org.firstinspires.ftc.teamcode.drive.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp (group = "Demo")
public class ServoTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        Servo leftFourBar, rightFourBar, intake;
        leftFourBar = drive.leftFourBar;
        rightFourBar = drive.rightFourBar;

        waitForStart();
        while (!isStopRequested()) {

            if (gamepad1.x) {
                leftFourBar.setPosition(0);
                rightFourBar.setPosition(0);
            }
            if (gamepad1.a) {
                leftFourBar.setPosition(.5);
                rightFourBar.setPosition(.5);
            }
            if (gamepad1.b) {
                leftFourBar.setPosition(1);
                rightFourBar.setPosition(1);
            }

        }
    }
}