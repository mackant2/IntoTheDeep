package org.firstinspires.ftc.teamcode.drive.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.custom.PressEventSystem;


@TeleOp (group = "tuning", name="[TUNING] Lift")
public class Lift extends LinearOpMode {
    float clamp(float num, float min, float max) {
        return Math.max(min, Math.min(num, max));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx liftLeft = drive.liftLeft;

        while (!isStopRequested()) {
            double power = gamepad1.right_trigger - gamepad1.left_trigger;
            int pos = (int)(liftLeft.getCurrentPosition() + Math.floor(power * 50));
            liftLeft.setTargetPosition(pos);

            telemetry.addData("Power", power);
            telemetry.addData("Current Position", liftLeft.getCurrentPosition());
            telemetry.addData("Target Position", liftLeft.getTargetPosition());
            telemetry.addData("Touching Limiter", drive.liftLimiter.isPressed());
            telemetry.addData("Mode", liftLeft.getMode());
            if (drive.liftLimiter.isPressed() && liftLeft.getCurrentPosition() != 0) {
                liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            telemetry.update();
        }
    }
}