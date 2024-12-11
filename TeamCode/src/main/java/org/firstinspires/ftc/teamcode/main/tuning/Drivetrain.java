package org.firstinspires.ftc.teamcode.main.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.main.utils.ParsedHardwareMap;


@TeleOp (group = "tuning", name="[TUNING] Drivetrain")
public class Drivetrain extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        ParsedHardwareMap parsedHardwareMap = new ParsedHardwareMap(hardwareMap);
        DcMotorEx frontLeft, frontRight, backLeft, backRight;
        frontLeft = parsedHardwareMap.frontLeft;
        frontRight = parsedHardwareMap.frontRight;
        backLeft = parsedHardwareMap.backLeft;
        backRight = parsedHardwareMap.backRight;

        frontLeft.setPower(0.5);
        frontRight.setPower(0.5);
        backLeft.setPower(0.5);
        backRight.setPower(0.5);

        while (!isStopRequested()) {
            telemetry.addData("Left front", frontLeft.getCurrentPosition());
            telemetry.addData("Back right", backRight.getCurrentPosition());
            telemetry.addData("Front right (Perp)", frontRight.getCurrentPosition());
            telemetry.update();
        }
    }
}