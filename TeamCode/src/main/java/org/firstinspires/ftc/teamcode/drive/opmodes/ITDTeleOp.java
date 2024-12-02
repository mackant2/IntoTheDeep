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
        arm.stateMachine.setState(Arm.ArmState.InitiatingTransfer);
    }

    void Arm_TransferInitCompleted() {
        intake.state = Intake.IntakeState.Transferring;
    }

    void Intake_TransferCompleted() {
        arm.stateMachine.setState(Arm.ArmState.Extracting);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        driverController = gamepad1;
        assistantController = gamepad2;
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);
        PressEventSystem pressEventSystem = new PressEventSystem(telemetry);
        Logger logger = new Logger();
        logger.Initialize(telemetry);
        intake = new Intake(drive, this, logger, this::Intake_TransferCompleted);
        Drivetrain drivetrain = new Drivetrain(drive, driverController);
        arm = new Arm(drive, telemetry, assistantController, this::Arm_TransferInitCompleted);

        logger.Initialize(telemetry);

        waitForStart();

        //initialize four bar
        arm.Initialize();
        //flip intake up and bring in
        intake.Initialize();

        pressEventSystem.AddListener(driverController, "a", arm::ToggleClaw);
        pressEventSystem.AddListener(driverController, "right_bumper", intake::ToggleAutomatedIntake);
        pressEventSystem.AddListener(driverController, "left_bumper", () -> arm.stateMachine.setState(Arm.ArmState.InitiatingTransfer));
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