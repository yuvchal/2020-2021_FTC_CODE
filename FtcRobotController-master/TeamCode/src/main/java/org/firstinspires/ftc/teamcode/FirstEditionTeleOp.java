package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="Test Teleop", group="BionicBot")
public class FirstEditionTeleOp extends LinearOpMode {


HardwareBionicbot robot = new HardwareBionicbot();

public void runOpMode() {
    robot.init(hardwareMap);
    waitForStart();

    while (opModeIsActive()) {
        double turn = gamepad1.right_stick_x;
        double lateral = gamepad1.left_stick_x;
        double forward = gamepad1.left_stick_y;
        double leftDrive, leftBack, rightDrive, rightBack;

        leftDrive = forward - lateral + turn;
        leftBack = forward + lateral + turn;
        rightDrive = forward + lateral + turn;
        rightBack = forward - lateral - turn;

        telemetry.addData("Controller X:", gamepad1.left_stick_x);
        telemetry.addData("Controller Y:", gamepad1.left_stick_y);
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);

        /*if(gamepad2.left_trigger >0)
        {
            robot.pincher.setPower(gamepad2.left_trigger);
        }
        else if(gamepad2.right_trigger > 0)
        {
            robot.pincher.setPower(-gamepad2.right_trigger);
        }
        else
        {
            robot.pincher.setPower(0);
        }
        if(gamepad2.left_stick_y>0) {
            robot.straight.setPower(gamepad2.left_stick_y);
        }
        else if(gamepad2.left_stick_y<0){
            robot.straight.setPower(gamepad2.left_stick_y);
        }
        else {
            robot.straight.setPower(.7);
        }
        if(gamepad2.right_bumper)
        {
            robot.lifter1.setPower(-.5);
            robot.lifter.setPower(-.5);
        }
        else if(gamepad2.left_bumper)
        {
            robot.lifter1.setPower(.5);
            robot.lifter.setPower(.5);
        }
        else {
            robot.lifter1.setPower(-.05);
            robot.lifter.setPower(-.05);
        }*/
        if (gamepad1.dpad_left) {
            robot.leftDrive.setPower(.5);
            robot.rightDrive.setPower(.5);
            robot.leftBack.setPower(-.5);
            robot.rightBack.setPower(-.5);
        } else if (gamepad1.dpad_right) {
            robot.leftDrive.setPower(-.5);
            robot.rightDrive.setPower(-.5);
            robot.leftBack.setPower(.5);
            robot.rightBack.setPower(.5);
        } else {
            robot.leftDrive.setPower(-leftDrive);
            robot.rightDrive.setPower(-rightDrive);
            robot.rightBack.setPower(rightBack);
            robot.leftBack.setPower(-leftBack);
        }
        if (gamepad1.left_trigger > 0)
        {
            robot.leftDrive.setPower(gamepad1.left_trigger);
            robot.rightDrive.setPower(-gamepad1.left_trigger);
            robot.leftBack.setPower(gamepad1.left_trigger);
            robot.rightBack.setPower(-gamepad1.left_trigger);
        }
        else if(gamepad1.right_trigger > 0)
        {
            robot.leftDrive.setPower(-gamepad1.right_trigger);
            robot.rightDrive.setPower(gamepad1.right_trigger);
            robot.leftBack.setPower(-gamepad1.right_trigger);
            robot.rightBack.setPower(gamepad1.right_trigger);
        }

        if(gamepad1.right_bumper)
            robot.intake.setPower(1);

        if(gamepad1.left_bumper)
            robot.intake.setPower(-1);

        while(gamepad2.y)
            robot.intake.setPower(1);

        while(gamepad2.x)
            robot.intake.setPower(-1);

        if(gamepad2.right_bumper) {
            robot.bottomSlider.setPower(-1);
            robot.topSlider.setPower(-1);
        }

        if(gamepad2.left_bumper) {
            robot.bottomSlider.setPower(1);
            robot.topSlider.setPower(1);
        }

        while(gamepad2.left_trigger > 0) {
            robot.leftShooter.setPower(-gamepad2.left_trigger);
            robot.rightShooter.setPower(gamepad2.left_trigger);
        }
        while(gamepad2.right_trigger > 0) {
            robot.leftShooter.setPower(gamepad2.right_trigger);
            robot.rightShooter.setPower(-gamepad2.right_trigger);
        }
        if(gamepad2.dpad_up)
            robot.wobblyJoint.setPower(-1);
        if(gamepad2.dpad_down)
            robot.wobblyClaw.setPosition(-0.8);
        else
            robot.wobblyClaw.setPosition(0);




        if (gamepad2.b){
            robot.planeCrosser.setPosition(0.8);
        }
        else if (gamepad2.a)
        {
            robot.planeCrosser.setPosition(-0.8);
        }
    }
}
}