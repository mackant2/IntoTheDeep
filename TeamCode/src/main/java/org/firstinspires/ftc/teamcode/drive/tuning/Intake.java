package org.firstinspires.ftc.teamcode.drive.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;


@TeleOp (group = "tuning", name = "[TUNING] Intake")
public class Intake extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx extender = drive.extendo;
        Servo flipdown = drive.flipdown;

        String state = "tuning";

        flipdown.setPosition(0);
        waitForStart();
        while (!isStopRequested()) {
            if (state.equals("tuning")) {
                telemetry.addLine("Move four bar in and press A/X when done");
                if (gamepad1.a) {
                    state = "resetting";
                }
            }
            else if (state.equals("resetting")) {
                if (extender.getMode() != DcMotor.RunMode.STOP_AND_RESET_ENCODER) {
                    extender.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                if (extender.getCurrentPosition() == 0) {
                    state = "completed";
                    requestOpModeStop();
                }
                telemetry.addLine("Resetting encoder...");
            }
            telemetry.update();
        }
    }
}