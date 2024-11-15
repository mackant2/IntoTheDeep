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
    Servo intake;
    intake = hardwareMap.get(Servo.class, "intake");

    waitForStart();
    while (!isStopRequested()) {
        if (gamepad1.right_bumper) {
            intake.setPosition(0);
        } else if (gamepad1.left_bumper) {
            intake.setPosition(1);
        } else {
            intake.setPosition(0);
        }


    }

}
}
