package org.firstinspires.ftc.teamcode.drive.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.custom.DelaySystem;


@TeleOp (group = "tuning", name="[TUNING] Lift")
public class Lift extends LinearOpMode {
    float clamp(float num, float min, float max) {
        return Math.max(min, Math.min(num, max));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DelaySystem delaySystem = new DelaySystem();
        DcMotorEx liftLeft = drive.liftLeft;

        float startTime = System.currentTimeMillis();
        float endTime = System.currentTimeMillis();

        boolean setting = false;

        while (!isStopRequested()) {
            if (!drive.liftLimiter.isPressed()) {
                liftLeft.setTargetPosition(liftLeft.getCurrentPosition() - 10);
                endTime = System.currentTimeMillis();
            }
            else if (liftLeft.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                delaySystem.CreateDelay(5000, this::requestOpModeStop);
                setting = true;
            }

            telemetry.addData("Time to tune", (endTime - startTime) / 1000);
            if (setting) {
                telemetry.addLine("Resetting encoder...");
            }
            telemetry.update();
        }
    }
}