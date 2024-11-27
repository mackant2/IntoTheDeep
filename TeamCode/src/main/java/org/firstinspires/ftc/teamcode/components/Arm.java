package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Arm {
    public enum ArmState {
            Moving,
            Transferring
    }
    public ArmState state = ArmState.Moving;
    Telemetry telemetry;
    Gamepad driverController;
    DcMotorEx liftLeft, liftRight;
    Servo leftFourBar, rightFourBar, wrist, claw;
    final double FOURBAR_ROTATE_SPEED = 0.02;
    final double CLAW_OPEN = 0.5;
    final double CLAW_CLOSED = 0.2;

    public static class Height {
        public static int LOWER_BUCKET = 0;
        public static int UPPER_BUCKET = 0;
        public static int LOWER_BAR = 0;
        public static int UPPER_BAR = 0;
        public static int DOWN = 0;
    }

    void RotateFourBar(double position) {
        leftFourBar.setPosition(position);
        rightFourBar.setPosition(position);
    }

    public Arm(SampleMecanumDrive drive, Telemetry telemetry, Gamepad driverController) {
        this.driverController = driverController;
        this.telemetry = telemetry;

        //assign motors
        liftLeft = drive.liftLeft;
        liftRight = drive.liftRight;
        //assign servos
        //four bar - 0 is out, 1 is transfer position
        leftFourBar = drive.leftFourBar;
        rightFourBar = drive.rightFourBar;
        wrist = drive.wrist;
        claw = drive.claw;
    }

    public void Initialize() {
        //initialize arm to transfer
        //liftRight.setVelocity(200);
        //liftLeft.setVelocity(200);
        GoToHeight(50);
        claw.setPosition(CLAW_OPEN);
    }

    public void ToggleClaw() {
        if (state == ArmState.Moving) {
            claw.setPosition(claw.getPosition() == CLAW_OPEN ? CLAW_CLOSED : CLAW_OPEN);
        }
    }

    public void GoToHeight(int height) {
        //liftLeft.setTargetPosition(height);
        //liftRight.setTargetPosition(height);
    }

    void RotateWrist(double position) {
        wrist.setPosition(position);
    }

    public boolean IsMoving() {
        return liftRight.isBusy();
    }

    float clamp(float num, float min, float max) {
        return Math.max(min, Math.min(num, max));
    }

    public void Update() {
        if (state == ArmState.Moving) {
            float armUp = driverController.right_trigger;
            float armDown = driverController.left_trigger;
            float input = armUp - armDown;
            int currentPosition = liftRight.getCurrentPosition();
            //Math.clamp causes crash here, so using custom method
            int targetPosition = (int)clamp(liftRight.getTargetPosition() + input * 20, 50, 4000);
            if (currentPosition != targetPosition) {
                GoToHeight(targetPosition);
            }

            float dist = targetPosition - currentPosition;
            float delta = Math.signum(dist);
            //AdjustLiftVelocity(clamp(clamp(dist / 250, -1, 1), delta * (float)0.001, delta));

            double leftPos = leftFourBar.getPosition();
            double change = 0;
            if (driverController.dpad_left) {
                change -= FOURBAR_ROTATE_SPEED;
            }
            if (driverController.dpad_right) {
                change += FOURBAR_ROTATE_SPEED;
            }
            //Math.clamp causes crash here, so using custom method
            double leftClamped = clamp((float)(leftPos + change), (float)CLAW_OPEN, (float)CLAW_CLOSED);
            if (change != 0) {
                RotateFourBar(leftClamped);
            }

            if (driverController.dpad_up) {
                change -= FOURBAR_ROTATE_SPEED;
            }
            if (driverController.dpad_down) {
                change += FOURBAR_ROTATE_SPEED;
            }

            /*float clamped = (clamp(targetPosition, 50, 400) - 50) / 350;
            RotateFourBar(clamp(clamped, 0, (float)0.6));
            wrist.setPosition(clamped);
            telemetry.addData("pos", leftFourBar.getPosition());*/
        }

        telemetry.addData("Arm State", state);
        telemetry.addData("Arm Wrist Position", wrist.getPosition());
        telemetry.addData("Arm Claw Position", claw.getPosition());
        telemetry.addData("Lift Velocity", liftRight.getVelocity());
        telemetry.addData("Lift Current Position", liftRight.getCurrentPosition());
        telemetry.addData("Lift Target Position", liftRight.getTargetPosition());
    }
}