package org.firstinspires.ftc.teamcode.drive.opmodes;

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

@TeleOp (name = "[OFFICIAL] TeleOp", group = "official")
public class ITDTeleOp extends LinearOpMode {
    Servo claw;
    Gamepad driverController;
    Gamepad assistantController;
    Intake intake;
    Arm arm;

    void Transfer() {
        arm.state = Arm.ArmState.Transferring;
        intake.state = Intake.IntakeState.Transferring;
    }

    @Override
    public void runOpMode() throws InterruptedException {
        driverController = gamepad1;
        assistantController = gamepad2;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        PressEventSystem pressEventSystem = new PressEventSystem(telemetry);
        Logger logger = new Logger();
        logger.Initialize(telemetry);
        intake = new Intake(drive, this, logger);
        Drivetrain drivetrain = new Drivetrain(drive, driverController);
        arm = new Arm(drive, telemetry, driverController);

        logger.Initialize(telemetry);

        waitForStart();

        //initialize four bar to transfer
        arm.Initialize();
        intake.Initialize();

        //Toggle claw with A press - driver
        pressEventSystem.AddListener(driverController, "a", () -> arm.ToggleClaw());
        //Run intake with right bumper press - assistant
        pressEventSystem.AddListener(driverController, "right_bumper", () -> intake.state = intake.state == Intake.IntakeState.RunningAutomatedIntake ? Intake.IntakeState.Idle : Intake.IntakeState.RunningAutomatedIntake);
        pressEventSystem.AddListener(driverController, "left_bumper", this::Transfer);
        while (!isStopRequested()) {
            //Update utils
            pressEventSystem.Update();
            //Update components
            drivetrain.Update();
            arm.Update();
            intake.Update();

            telemetry.update();
        }
    }
}