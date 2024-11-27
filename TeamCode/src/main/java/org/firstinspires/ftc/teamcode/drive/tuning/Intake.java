package org.firstinspires.ftc.teamcode.drive.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp (group = "tuning", name = "[TUNING] Intake")
public class Intake extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx intake = drive.intake;
        DcMotorEx extender = drive.extendo;

        waitForStart();
        while (!isStopRequested()) {
            intake.setPower(gamepad1.a ? -0.2 : 0);
            telemetry.addData("intake", intake.getPower());
            telemetry.addData("intake pos", intake.getCurrentPosition());

            extender.setPower((gamepad1.left_trigger  - gamepad1.right_trigger) * 0.2);
            telemetry.addData("extender position", extender.getCurrentPosition());

            telemetry.update();
        }
    }
}