package org.firstinspires.ftc.teamcode.drive.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp (group = "tuning", name="[TUNING] Lift")
public class Lift extends LinearOpMode {
    float clamp(float num, float min, float max) {
        return Math.max(min, Math.min(num, max));
    }

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx liftLeft = drive.liftLeft;

        float startTime = System.currentTimeMillis();
        float endTime;

        String state = "raising";

        while (!isStopRequested()) {
            if (state.equals("raising")) {
                telemetry.addLine("Please raise the lift using the right trigger to the desired height.");
                telemetry.addLine("Press A when you are ready to begin the automated tuning process.");

                liftLeft.setTargetPosition(liftLeft.getTargetPosition() + Math.round(gamepad1.right_trigger * 10));

                if (gamepad1.a) {
                    state = "tuning";
                }
            }
            else if (state.equals("tuning")) {
                liftLeft.setTargetPosition(liftLeft.getCurrentPosition() - 10);
                endTime = System.currentTimeMillis();

                if (drive.liftLimiter.isPressed()) {
                    state = "resetting";
                }

                telemetry.addData("Time to tune", (endTime - startTime) / 1000);
            }
            else if (state.equals("resetting")) {
                if (liftLeft.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                    liftLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                if (liftLeft.getCurrentPosition() == 0) {
                    state = "complete";
                    requestOpModeStop();
                }
                telemetry.addLine("Resetting encoder...");
            }
            telemetry.update();
        }
    }
}