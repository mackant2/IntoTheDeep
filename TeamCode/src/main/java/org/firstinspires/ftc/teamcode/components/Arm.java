package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.sfdev.assembly.state.StateMachine;
import com.sfdev.assembly.state.StateMachineBuilder;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.util.custom.DelaySystem;

public class Arm {
    public enum ArmState {
        DriverControlled,
        InitiatingTransfer,
        WaitingForSample,
        Extracting
    }
    static class FourBarPosition {
        public static final double Transfer = 0.26;
    }
    static class WristPosition {
        public static final double Transfer = 0.5;
    }
    public static class Height {
        public static final int LOWER_BUCKET = 2500;
        public static final int UPPER_BUCKET = 3500;
        public static final int LOWER_BAR = 1500;
        public static final int UPPER_BAR = 2500;
        public static final int DOWN = 0;
        public static final int ExtractionComplete = 400;
    }
    Telemetry telemetry;
    Gamepad assistantController;
    DcMotorEx liftLeft, liftRight;
    Servo leftFourBar, rightFourBar, wrist, claw;
    final double FOURBAR_ROTATE_SPEED = 0.02;
    final double CLAW_OPEN = 0.8;
    final double CLAW_CLOSED = 0.3;
    public StateMachine stateMachine;
    ArmEvent transferInitCompleted;

    public interface ArmEvent {
        void fire();
    }

    void RotateFourBar(double position) {
        leftFourBar.setPosition(position);
        rightFourBar.setPosition(position);
    }

    public Arm(SampleMecanumDrive drive, Telemetry telemetry, Gamepad assistantController, ArmEvent transferInitCompleted) {
        this.assistantController = assistantController;
        this.telemetry = telemetry;
        this.transferInitCompleted = transferInitCompleted;

        //assign motors
        liftLeft = drive.liftLeft;
        liftRight = drive.liftRight;
        //assign servos
        //four bar - 0 is out, 1 is transfer position
        leftFourBar = drive.leftFourBar;
        rightFourBar = drive.rightFourBar;
        wrist = drive.wrist;
        claw = drive.claw;

        stateMachine = new StateMachineBuilder()
                .state(ArmState.DriverControlled)
                .transition(() -> stateMachine.getState() == ArmState.InitiatingTransfer)
                .state(ArmState.InitiatingTransfer)
                .transition(() -> liftLeft.getCurrentPosition() <= 10 && leftFourBar.getPosition() == FourBarPosition.Transfer)
                .state(ArmState.WaitingForSample)
                .transition(() -> stateMachine.getState() == ArmState.Extracting)
                .state(ArmState.Extracting)
                .transition(() -> leftFourBar.getPosition() == 1, ArmState.DriverControlled)
                .build();
    }

    public void Initialize() {
        stateMachine.start();
        GoToHeight(0);
        claw.setPosition(CLAW_OPEN);
    }

    public void ToggleClaw() {
        if (stateMachine.getState() == ArmState.DriverControlled) {
            claw.setPosition(claw.getPosition() == CLAW_OPEN ? CLAW_CLOSED : CLAW_OPEN);
        }
    }

    public void GoToHeight(int height) {
        liftLeft.setTargetPosition(height);
        //liftRight.setTargetPosition(height);
    }

    public boolean IsMoving() {
        return liftLeft.isBusy();
    }

    float clamp(float num, float min, float max) {
        return Math.max(min, Math.min(num, max));
    }

    boolean transferInitiated = false;
    boolean extractingEntered = false;

    public void Update() {
        stateMachine.update();
        ArmState state = (ArmState)stateMachine.getState();

        if (state == ArmState.DriverControlled) {
            double power = assistantController.right_trigger - assistantController.left_trigger;
            if (power != 0) {
                int pos = (int)clamp((float)(liftLeft.getCurrentPosition() + Math.floor(power * 50)), 0, 2500);
                liftLeft.setTargetPosition(pos);
            }

            double leftPos = leftFourBar.getPosition();
            double change = 0;
            if (assistantController.dpad_left) {
                change -= FOURBAR_ROTATE_SPEED;
            }
            if (assistantController.dpad_right) {
                change += FOURBAR_ROTATE_SPEED;
            }
            //Math.clamp causes crash here, so using custom method
            double leftClamped = clamp((float)(leftPos + change), (float)FourBarPosition.Transfer, 1);
            if (change != 0) {
                RotateFourBar(leftClamped);
            }
        }
        else if (state == ArmState.InitiatingTransfer) {
            if (!transferInitiated) {
                transferInitiated = true;
                GoToHeight(Height.DOWN);
                wrist.setPosition(WristPosition.Transfer);
                RotateFourBar(FourBarPosition.Transfer);
            }
        }
        else if (state == ArmState.WaitingForSample) {
            if (transferInitiated) {
                //reset states so transfer can happen again
                transferInitiated = false;
                claw.setPosition(CLAW_OPEN);
                transferInitCompleted.fire();
            }
        }
        else if (state == ArmState.Extracting) {
            if (!extractingEntered) {
                //reset state
                extractingEntered = true;
                claw.setPosition(CLAW_CLOSED);
                liftLeft.setVelocity(500);
                GoToHeight(Height.ExtractionComplete);
            }
            double completionPercentage = clamp((float)((double)liftLeft.getCurrentPosition() / Height.ExtractionComplete), 0, 1);
            //Move both four bar and wrist to 0 throughout sample extraction from transfer plate
            RotateFourBar(FourBarPosition.Transfer - completionPercentage * FourBarPosition.Transfer);
            wrist.setPosition(WristPosition.Transfer - completionPercentage * WristPosition.Transfer);
            if (completionPercentage >= 1) {
                extractingEntered = false;
                RotateFourBar(1);
                wrist.setPosition(0.2);
                liftLeft.setVelocity(1000);
            }
        }

        telemetry.addData("Arm State", state);
        telemetry.addData("Lift Position", liftLeft.getCurrentPosition());
        telemetry.addData("Lift Target", liftLeft.getTargetPosition());
        telemetry.addData("Four Bar Position", leftFourBar.getPosition());
    }
}