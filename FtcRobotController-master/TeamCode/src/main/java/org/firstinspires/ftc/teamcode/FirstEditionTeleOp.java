package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Test Teleop", group="BionicBot")
public class FirstEditionTeleOp extends LinearOpMode {


HardwareBionicbot robot = new HardwareBionicbot();
    BNO055IMU imu;

    Orientation angles;
    Orientation newAngles;

public void runOpMode() {
    BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
    parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
    imu = hardwareMap.get(BNO055IMU.class,"imu");
    imu.initialize(parameters);

    robot.init(hardwareMap);
    waitForStart();

    while (opModeIsActive()) {

        getDateImu();
        alignGyro();
        //shootAlignGyro();
        shoot2AlignGyro();


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

        //Driving
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
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.rightBack.setPower(0);
            robot.leftBack.setPower(0);
        }
        if(gamepad1.right_stick_x < 0)
        {
            robot.leftDrive.setPower(-gamepad1.right_stick_x);
            robot.rightDrive.setPower(-gamepad1.right_stick_x);
            robot.rightBack.setPower(-gamepad1.right_stick_x);
            robot.leftBack.setPower(-gamepad1.right_stick_x);
        }
        else if(gamepad1.right_stick_x > 0)
        {
            robot.leftDrive.setPower(-gamepad1.right_stick_x);
            robot.rightDrive.setPower(-gamepad1.right_stick_x);
            robot.rightBack.setPower(-gamepad1.right_stick_x);
            robot.leftBack.setPower(-gamepad1.right_stick_x);
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
        else if(gamepad1.left_bumper)
            robot.intake.setPower(-1);
        else
            robot.intake.setPower(0);

        if(gamepad2.y)
            robot.intake.setPower(1);
        else if(gamepad2.x)
            robot.intake.setPower(-1);
        else
            robot.intake.setPower(0);


        if(gamepad2.right_bumper) {
            robot.bottomSlider.setPower(-1);
            robot.topSlider.setPower(-1);
        }
        else if(gamepad2.left_bumper) {
            robot.bottomSlider.setPower(1);
            robot.topSlider.setPower(1);
        }
        else {
            robot.bottomSlider.setPower(0);
            robot.topSlider.setPower(0);
        }


        if(gamepad2.left_trigger > 0) {
            robot.leftShooter.setPower(1);
            robot.rightShooter.setPower(-1);
        }
        else if(gamepad2.right_trigger > 0) {
            robot.leftShooter.setPower(1);
            robot.rightShooter.setPower(-1);
        }
        else {
            robot.leftShooter.setPower(0);
            robot.rightShooter.setPower(0);
        }
        if(gamepad2.dpad_left)
            robot.wobblyJoint.setPower(.5);
        else if(gamepad2.dpad_right)
            robot.wobblyJoint.setPower(-.5);
        else
            robot.wobblyJoint.setPower(0);
        if(gamepad2.a)
            robot.wobblyClaw.setPosition(1);
        else if(gamepad2.b)
            robot.wobblyClaw.setPosition(-1);
        }


}
    public void getDateImu()
    {
        angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        telemetry.addData("Heading", angles.firstAngle);
        telemetry.addData("Roll", angles.secondAngle);
        telemetry.addData("Pitch",angles.thirdAngle);
        telemetry.update();
    }
    public void GyroLeft()
    {
        while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 90) {
            newAngles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);

            if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 25){
                robot.leftDrive.setPower(1);
                robot.rightDrive.setPower(1);
                robot.rightBack.setPower(1);
                robot.leftBack.setPower(1);
            }
            else{
                robot.leftDrive.setPower(0.2);
                robot.rightDrive.setPower(0.2);
                robot.rightBack.setPower(0.2);
                robot.leftBack.setPower(0.2);
            }
            telemetry.addData("Heading", newAngles.firstAngle);
            telemetry.addData("Roll", newAngles.secondAngle);
            telemetry.addData("Pitch",newAngles.thirdAngle);
            telemetry.update();
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftBack.setPower(0);
    }
    public void GyroRight()
    {
        while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > -90) {
            newAngles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);

            if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > -25){
                robot.leftDrive.setPower(-1);
                robot.rightDrive.setPower(-1);
                robot.rightBack.setPower(-1);
                robot.leftBack.setPower(-1);
            }
            else{
                robot.leftDrive.setPower(-0.2);
                robot.rightDrive.setPower(-0.2);
                robot.rightBack.setPower(-0.2);
                robot.leftBack.setPower(-0.2);
            }
            telemetry.addData("Heading", newAngles.firstAngle);
            telemetry.addData("Roll", newAngles.secondAngle);
            telemetry.addData("Pitch",newAngles.thirdAngle);
            telemetry.update();
        }
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.rightBack.setPower(0);
        robot.leftBack.setPower(0);
    }
    public void shootAlignGyro()
    {
        while(gamepad1.b)
        {
            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > 0) {
                newAngles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);

                if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > 25){
                    robot.leftDrive.setPower(-1);
                    robot.rightDrive.setPower(-1);
                    robot.rightBack.setPower(-1);
                    robot.leftBack.setPower(-1);
                }
                else
                {
                    robot.leftDrive.setPower(-0.2);
                    robot.rightDrive.setPower(-0.2);
                    robot.rightBack.setPower(-0.2);
                    robot.leftBack.setPower(-0.2);
                }
            }
            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 0) {
                if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < -35) {
                    robot.leftDrive.setPower(1);
                    robot.rightDrive.setPower(1);
                    robot.rightBack.setPower(1);
                    robot.leftBack.setPower(1);
                }
                else if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 0)
                {
                    robot.leftDrive.setPower(.2);
                    robot.rightDrive.setPower(.2);
                    robot.rightBack.setPower(.2);
                    robot.leftBack.setPower(.2);
                }


            }
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.rightBack.setPower(0);
            robot.leftBack.setPower(0);
        }
    }
    public void alignGyro()
    {
        s:while(gamepad1.x)
        {
            if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > 0) {
                while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > 0) {
                    robot.leftDrive.setPower(-.1);
                    robot.rightDrive.setPower(-.1);
                    robot.rightBack.setPower(-.1);
                    robot.leftBack.setPower(-.1);
                }
                break s;
            }
            else if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 0) {
                while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 0) {
                    robot.leftDrive.setPower(.1);
                    robot.rightDrive.setPower(.1);
                    robot.rightBack.setPower(.1);
                    robot.leftBack.setPower(.1);
                }
                break s;
            }
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.rightBack.setPower(0);
            robot.leftBack.setPower(0);
            sleep(1000);

        }
    }
    public void shoot2AlignGyro()
    {
        s:while(gamepad1.b)
        {
           if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > 0)
               shootPositive();

           else if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 0)
               shootNegative();
           else {
               robot.leftDrive.setPower(0);
               robot.rightDrive.setPower(0);
               robot.rightBack.setPower(0);
               robot.leftBack.setPower(0);
           }
        }
    }
    public void shootPositive()
    {
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > 0) {
            newAngles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);

            if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > 25){
                robot.leftDrive.setPower(-1);
                robot.rightDrive.setPower(-1);
                robot.rightBack.setPower(-1);
                robot.leftBack.setPower(-1);
            }
            else
            {
                robot.leftDrive.setPower(-0.2);
                robot.rightDrive.setPower(-0.2);
                robot.rightBack.setPower(-0.2);
                robot.leftBack.setPower(-0.2);
            }

        }
        if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < -0.5)
            shootNegative();
        else {
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.rightBack.setPower(0);
            robot.leftBack.setPower(0);

        }

    }
    public void shootNegative()
    {
        while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 0) {
            if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < -35) {
                robot.leftDrive.setPower(1);
                robot.rightDrive.setPower(1);
                robot.rightBack.setPower(1);
                robot.leftBack.setPower(1);
            }
            else if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 0)
            {
                robot.leftDrive.setPower(.2);
                robot.rightDrive.setPower(.2);
                robot.rightBack.setPower(.2);
                robot.leftBack.setPower(.2);
            }

        }
        if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > 0.5)
            shootPositive();
        else {
            robot.leftDrive.setPower(0);
            robot.rightDrive.setPower(0);
            robot.rightBack.setPower(0);
            robot.leftBack.setPower(0);

        }
    }
}