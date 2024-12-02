package org.firstinspires.ftc.teamcode.drive.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.custom.PressEventSystem;


@TeleOp (group = "tuning", name = "[TUNING] Intake")
public class Intake extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        PressEventSystem pressEventSystem = new PressEventSystem(telemetry);
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        DcMotorEx intake = drive.intake;
        DcMotorEx extender = drive.extendo;
        Servo flipdown = drive.flipdown;

        waitForStart();
        flipdown.setPosition(0);
        pressEventSystem.AddListener(gamepad1, "right_bumper", () -> flipdown.setPosition(flipdown.getPosition() == 0 ? 1 : 0));
        while (!isStopRequested()) {
            intake.setPower(gamepad1.a ? -0.5 : 0);

            double power = gamepad1.right_trigger - gamepad1.left_trigger;
            if (power != 0) {
                int currentPosition = extender.getCurrentPosition();
                int change = currentPosition - (int)(power * 10);
                extender.setTargetPosition(currentPosition + change);
            }

            pressEventSystem.Update();

            telemetry.addData("Extender Position", extender.getCurrentPosition());
            telemetry.update();
        }
    }
}