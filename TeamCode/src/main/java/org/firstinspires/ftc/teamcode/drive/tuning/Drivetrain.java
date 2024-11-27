package org.firstinspires.ftc.teamcode.drive.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp (group = "tuning", name="[TUNING] Drivetrain")
public class Drivetrain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx frontLeft, frontRight, backLeft, backRight;
        frontLeft = drive.frontLeft;
        frontRight = drive.frontRight;
        backLeft = drive.backLeft;
        backRight = drive.backRight;

        while (!isStopRequested()) {
            double leftVelocity = gamepad1.left_trigger * 200;
            double rightVelocity = gamepad1.right_trigger * 200;
            frontLeft.setVelocity(leftVelocity);
            backLeft.setVelocity(leftVelocity);
            frontRight.setVelocity(rightVelocity);
            backRight.setVelocity(rightVelocity);
        }
    }
}