package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Test Autonomous", group="BionicBot")
public class TestAutonomous1 extends LinearOpMode
{
    HardwareBionicbot         robot   = new HardwareBionicbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    static final double     COUNTS_PER_MOTOR_REV    = 1200  ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     CIRCUMFERENCE           = WHEEL_DIAMETER_INCHES * Math.PI;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (CIRCUMFERENCE);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;
    public void runOpMode()
    {
        robot.init(hardwareMap);

//        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        waitForStart();
        robot.wobblyJoint.setPower(1);
        sleep(1000);
    }
    public void DriveForwardDistance(double speed, double distanceInches)
    {
        double rotationsneeded = distanceInches/CIRCUMFERENCE;
        int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() - distanceTick);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + distanceTick);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() - distanceTick);
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() + distanceTick);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DriveForward(speed);

        while(robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() )
        {

        }
        StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void DriveBackwardDistance(double speed, double distanceInches)
    {
        double rotationsneeded = distanceInches/CIRCUMFERENCE;
        int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + distanceTick);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() - distanceTick);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() + distanceTick);
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() - distanceTick);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        DriveBackward(speed);

        while(robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() )
        {

        }
        StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void TurnLeftDistance(double speed, int distanceInches)
    {
        double rotationsneeded = distanceInches/CIRCUMFERENCE;
        int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + distanceTick);
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() + distanceTick);
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + distanceTick);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() + distanceTick);

        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        TurnLeft(speed);


        while( robot.rightDrive.isBusy() && robot.rightBack.isBusy() && robot.leftBack.isBusy() && robot.leftDrive.isBusy() )
        {

        }
        StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void TurnRightDistance(double speed, int distanceInches)
    {
        double rotationsneeded = distanceInches/CIRCUMFERENCE;
        int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() - distanceTick);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() - distanceTick);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() - distanceTick);
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() - distanceTick);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        TurnRight(speed);


        while(robot.leftDrive.isBusy()  && robot.leftBack.isBusy() && robot.rightDrive.isBusy()  && robot.rightBack.isBusy() )
        {

        }
        StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void StrafLeftDistance(double speed, int distanceInches)
    {
        double rotationsneeded = distanceInches/CIRCUMFERENCE;
        int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);

        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + distanceTick);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() - distanceTick);
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() - distanceTick);
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() + distanceTick);


        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        StrafLeft(speed);

        while(robot.rightDrive.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() && robot.leftDrive.isBusy()) {

        }

        StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void StrafRightDistance(double speed, int distanceInches)
    {
        double rotationsneeded = distanceInches/CIRCUMFERENCE;
        int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() + distanceTick);
        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() - distanceTick);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() - distanceTick);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() + distanceTick);



        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);




        StrafRight(speed);


        while(robot.leftDrive.isBusy() && robot.rightBack.isBusy() && robot.leftBack.isBusy() && robot.rightDrive.isBusy())
        {

        }
        StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void RightDiagonal(double speed, double distanceInches)
    {
        double rotationsneeded = distanceInches/CIRCUMFERENCE;
        int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition());
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition() + distanceTick);
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition() - distanceTick);
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition());

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DriveForward(speed);

        while(robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() )
        {

        }
        StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void LeftDiagonal(double speed, double distanceInches)
    {
        double rotationsneeded = distanceInches/CIRCUMFERENCE;
        int distanceTick = (int)(rotationsneeded*COUNTS_PER_MOTOR_REV);

        robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.leftDrive.setTargetPosition(robot.leftDrive.getCurrentPosition() - distanceTick);
        robot.rightDrive.setTargetPosition(robot.rightDrive.getCurrentPosition());
        robot.leftBack.setTargetPosition(robot.leftBack.getCurrentPosition());
        robot.rightBack.setTargetPosition(robot.rightBack.getCurrentPosition() + distanceTick);

        robot.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        DriveForward(speed);

        while(robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() )
        {

        }
        StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void revHex(double power, int tick)
    {
        robot.wobblyJoint.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.wobblyJoint.setTargetPosition(robot.wobblyJoint.getCurrentPosition() - tick);

        robot.wobblyJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.wobblyJoint.setPower(power);

        while(robot.wobblyJoint.isBusy())
        {

        }
        robot.wobblyJoint.setPower(0);

        robot.wobblyJoint.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void DriveForward(double power)
    {
        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(power);
        robot.rightBack.setPower(power);
        robot.leftBack.setPower(-power);
    }
    public void DriveBackward(double power)
    {
        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(-power);
        robot.rightBack.setPower(-power);
        robot.leftBack.setPower(-power);
    }
    public void TurnLeft(double power)
    {
        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(power);
        robot.rightBack.setPower(power);
        robot.leftBack.setPower(-power);
    }
    public void TurnRight(double power)
    {
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(-power);
        robot.rightBack.setPower(-power);
        robot.leftBack.setPower(power);
    }
    public void StrafLeft(double power)
    {
        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightBack.setPower(-power);
    }
    public void StrafRight(double power)
    {
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(-power);
        robot.leftBack.setPower(-power);
        robot.rightBack.setPower(power);
    }

    public void StopDriving()
    {
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(0);
        robot.leftBack.setPower(0);
        robot.rightBack.setPower(0);
    }
    public void DriveForward(double power, int sleepTime)
    {
        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(power);
        robot.rightBack.setPower(power);
        robot.leftBack.setPower(-power);
        sleep(sleepTime);
        StopDriving();
    }
    public void DriveBackward(double power, int sleepTime)
    {
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(-power);
        robot.rightBack.setPower(-power);
        robot.leftBack.setPower(power);
        sleep(sleepTime);
        StopDriving();
    }
    public void TurnLeft(double power, int sleepTime)
    {
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(power);
        robot.rightBack.setPower(power);
        robot.leftBack.setPower(power);
        sleep(sleepTime);
        StopDriving();
    }
    public void TurnRight(double power, int sleepTime)
    {
        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(-power);
        robot.rightBack.setPower(-power);
        robot.leftBack.setPower(-power);
        sleep(sleepTime);
        StopDriving();
    }
    public void StrafeRight(double power, int sleepTime)
    {
        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(-power);
        robot.leftBack.setPower(power);
        robot.rightBack.setPower(power);
        sleep(sleepTime);
        StopDriving();
    }
    public void StrafeLeft(double power, int sleepTime)
    {
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(power);
        robot.leftBack.setPower(-power);
        robot.rightBack.setPower(-power);
        sleep(sleepTime);
        StopDriving();
    }
    public void DownLeft(double power, int sleepTime)
    {
        robot.rightDrive.setPower(0);
        robot.leftBack.setPower(0);
        robot.leftDrive.setPower(power);
        robot.rightBack.setPower(-power);
        sleep(sleepTime);
        StopDriving();
    }
    public void DownRight(double power, int sleepTime)
    {
        robot.rightDrive.setPower(-power);
        robot.leftBack.setPower(power);
        robot.leftDrive.setPower(0);
        robot.rightBack.setPower(0);
        sleep(sleepTime);
        StopDriving();
    }
    public void UpLeft(double power, int sleepTime)
    {
        robot.rightDrive.setPower(power);
        robot.leftBack.setPower(-power);
        sleep(sleepTime);
        StopDriving();
    }
    public void UpRight(double power, int sleepTime)
    {
        robot.leftDrive.setPower(-power);
        robot.rightBack.setPower(power);
        sleep(sleepTime);
        StopDriving();
    }
    public void Shoot(double power, int sleepTime)
    {
        robot.leftShooter.setPower(-1);
        robot.rightShooter.setPower(1);
        robot.bottomSlider.setPower(1);
        robot.topSlider.setPower(1);
        sleep(sleepTime);
        StopDriving();
    }
}