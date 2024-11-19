package org.firstinspires.ftc.teamcode.components;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;

public class Arm {
    Telemetry telemetry;
    Gamepad driverController;
    DcMotorEx liftLeft, liftRight;
    Servo leftFourBar, rightFourBar, wrist;

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
    }

    public void Initialize() {
        //initialize four bar to transfer
        GoToHeight(50);
    }

    public void GoToHeight(int height) {
        liftLeft.setTargetPosition(height);
        liftRight.setTargetPosition(height);
    }

    void AdjustPower(float power) {
        liftRight.setPower(power);
        liftLeft.setPower(power);
    }

    public boolean IsMoving() {
        return liftRight.isBusy();
    }

    float clamp(float num, float min, float max) {
        return Math.max(min, Math.min(num, max));
    }

    public void Update() {
        float armUp = driverController.right_trigger;
        float armDown = driverController.left_trigger;
        float power = armUp - armDown;
        int currentPosition = liftRight.getCurrentPosition();
        //Math.clamp causes crash here, so using custom method
        int targetPosition = (int)clamp(liftRight.getTargetPosition() + power * 10, 50, 1200);
        if (currentPosition != targetPosition) {
            GoToHeight(targetPosition);
        }

        AdjustPower(clamp((float)(targetPosition - currentPosition) / 250, -1, 1));

        telemetry.addData("liftPos", liftRight.getCurrentPosition());
        telemetry.addData("liftTarget", targetPosition);

        boolean leftDpad = driverController.dpad_left;
        boolean rightDpad = driverController.dpad_right;

        /*double leftPos = leftFourBar.getPosition();
        double change = 0;
        if (leftDpad) {
            change -= 0.01;
        }
        if (rightDpad) {
            change += 0.01;
        }
        //Math.clamp causes crash here, so using custom method
        double leftClamped = Math.max(0, Math.min(leftPos + change, 1));
        if (change != 0) {
            RotateFourBar(leftClamped);
        }*/

        float clamped = (clamp(targetPosition, 50, 600) - 50) / 550;
        RotateFourBar(1 - clamped);
        wrist.setPosition(clamped);
        telemetry.addData("pos", leftFourBar.getPosition());
    }
}