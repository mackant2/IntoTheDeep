package org.firstinspires.ftc.teamcode.drive.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.components.Arm;
import org.firstinspires.ftc.teamcode.components.Logger;
import org.firstinspires.ftc.teamcode.components.Intake;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

@Autonomous
public class StateFactoryDemo extends LinearOpMode {
    enum States {
        Extending,
        Dropping,
        Retracting
    }

    @Override
    public void runOpMode() throws InterruptedException {
        Logger logger = new Logger();
        logger.Initialize(telemetry);

        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Arm arm = new Arm(drive, telemetry, gamepad1);
        Intake intake = new Intake(drive, logger);

        StateMachine stateMachine = new StateMachineBuilder()
                .state(States.Extending)
                .onEnter(() -> arm.GoToHeight(Arm.Height.UPPER_BUCKET))
                .transition(() -> !arm.IsMoving())
                .state(States.Dropping)
                .onEnter(() -> intake.SetRunning(true))
                .transition(() -> !intake.IsRunning())
                .state(States.Retracting)
                .onEnter(() -> arm.GoToHeight(Arm.Height.DOWN))
                .build();

        Telemetry.Item stateItem = telemetry.addData("State", stateMachine.getState());

        waitForStart();

        if (isStopRequested()) return;

        stateMachine.start();
        while (opModeIsActive()) {
            //loop
            stateMachine.update();
            stateItem.setValue(stateMachine.getState());
            telemetry.update();
        }
    }
}