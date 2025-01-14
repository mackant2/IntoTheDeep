package org.firstinspires.ftc.teamcode.main.components;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.main.utils.ParsedHardwareMap;

import org.firstinspires.ftc.teamcode.main.utils.PressEventSystem;
import org.firstinspires.ftc.teamcode.main.utils.Robot;

@TeleOp(name = "Intaketester", group = "official")
public class IntakeTester extends LinearOpMode {
    Gamepad driverController;
    Gamepad assistantController;
    Robot robot;
    ColorSensor intakeColorSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        ParsedHardwareMap parsedHardwareMap = new ParsedHardwareMap(hardwareMap);
        PressEventSystem pressEventSystem = new PressEventSystem(telemetry);

        waitForStart();

        robot = new Robot(this, parsedHardwareMap, true);

        if(gamepad1.dpad_left){
            robot.intake.SetIntakeState((Intake.IntakeState.Intaking));
        }
        if(gamepad1.dpad_right){
            robot.intake.SetIntakeState((Intake.IntakeState.Rejecting));
        }


    }
}