package org.firstinspires.ftc.teamcode.drive.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.custom.PressEventSystem;


@TeleOp (group = "tuning", name="[TUNING] Lift")
public class Lift extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx liftRight = drive.liftRight;
        DcMotorEx liftLeft = drive.liftLeft;

        liftRight.setVelocity(200);
        liftLeft.setVelocity(200);

        while (!isStopRequested()) {
            double change = (gamepad1.right_trigger - gamepad1.left_trigger) * 20;
            int target = (int)(liftRight.getCurrentPosition() + change);
            liftRight.setTargetPosition(target);
            liftLeft.setTargetPosition(target);

            telemetry.addData("Current Position", liftRight.getCurrentPosition());
            telemetry.addData("Target Position", target);
            telemetry.update();
        }
    }
}