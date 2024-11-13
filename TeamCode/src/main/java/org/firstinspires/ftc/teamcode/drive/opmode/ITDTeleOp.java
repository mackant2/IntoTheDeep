package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Drivetrain;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.components.Logger;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.custom.PressEventSystem;

@TeleOp (name = "ITD TeleOp")
public class ITDTeleOp extends LinearOpMode {
    Servo claw;
    boolean isClawOpen = false;
    Gamepad driverController;
    Gamepad assistantController;

    @Override
    public void runOpMode() throws InterruptedException {
        driverController = gamepad1;
        assistantController = gamepad2;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        PressEventSystem pressEventSystem = new PressEventSystem(telemetry);
        Logger logger = new Logger();
        logger.Initialize(telemetry);
        Intake intake = new Intake(drive, logger);
        Drivetrain drivetrain = new Drivetrain(drive, driverController);
        Arm arm = new Arm(drive, telemetry, driverController);

        claw = drive.claw;

        waitForStart();

        //initialize claw to open for transfer
        claw.setPosition(1);
        //initialize four bar to transfer
        arm.Initialize();

        //Toggle claw with A press - driver
        pressEventSystem.AddListener(driverController, "a", this::ToggleClaw);
        //Toggle intake with B press - driver
        pressEventSystem.AddListener(assistantController, "b", () -> intake.SetRunning(!intake.IsRunning()));

        while (!isStopRequested()) {
            //Update utils
            pressEventSystem.Update();
            //Update components
            intake.Update();
            drivetrain.Update();
            arm.Update();
        }
    }

    void ToggleClaw() {
        telemetry.update();
        isClawOpen = !isClawOpen;
        claw.setPosition(isClawOpen ? 0 : 1);
    }
}