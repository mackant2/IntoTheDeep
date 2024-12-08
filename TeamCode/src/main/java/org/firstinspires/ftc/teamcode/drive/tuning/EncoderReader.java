package org.firstinspires.ftc.teamcode.drive.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.util.custom.ParsedHardwareMap;
import org.firstinspires.ftc.teamcode.util.custom.PressEventSystem;


@TeleOp (group = "tuning", name="[TUNING] Encoder Reader")
public class EncoderReader extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx liftLeft = hardwareMap.get(DcMotorEx.class, "liftLeft");
        liftLeft.setPower(-0.5);
        while (!isStarted()) {
            telemetry.addData("Lift Position", liftLeft.getCurrentPosition());
            telemetry.update();
        }
    }
}