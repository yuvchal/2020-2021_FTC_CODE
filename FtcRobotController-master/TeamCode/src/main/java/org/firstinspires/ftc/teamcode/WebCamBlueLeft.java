/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.robotcontroller.external.samples.HardwarePushbot;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;

import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "BlueLeft", group = "BionicBot")
@Disabled
public class WebCamBlueLeft extends LinearOpMode {
    HardwareBionicbot robot   = new HardwareBionicbot();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();
    private static int diskLevel = 0;
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    static final double     COUNTS_PER_MOTOR_REV    = 537.6  ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     CIRCUMFERENCE           = WHEEL_DIAMETER_INCHES * Math.PI;
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (CIRCUMFERENCE);
    static final double     DRIVE_SPEED             = 0.6;
    static final double     TURN_SPEED              = 0.5;

    //    private static final String LABEL_ZERO_ELEMENT = "Zero";
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AbKEjVb/////AAABmSwPzXhWz0Bwg7Axk/cxi78PH6k6FiQXLibmed4RXGqHUt7H8qLkyMPl8sScmzk51liMhzXIous0vblrvSPvSwucVzWnLWo6EMgrLrGmAMN/wqF0Ws5czwmL+13Vwaqa2I8hTK+glBwsMwuSFug4SlgJirmAn3o9E0FWJN/2w8fl9TQNQI6wZXi96BjVqtN/X6kqR2Z2zQRpsKwqxrC3RaynsG6MCOcu+twOjWwl3auVM/i8m8yc57+JmHYDfKSW17Yb5yO+q+WQunNNMduKWHoGdaEKIbuwv3Vbq+XQC73w3xHKWb/6MqcgZydMW2HoE7FBhJxCpMP855/nFqkWTZlotS7SnYWoKarovpw+gKUv";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;
    BNO055IMU imu;

    Orientation angles;
    Orientation newAngles;
    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();


        if (tfod != null) {
            // getUpdatedRecognitions() will return null if no new information is available since
            // the last time that call was made.
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
            if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.

            }
        }telemetry.update();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;

        imu = hardwareMap.get(BNO055IMU.class,"imu");
        imu.initialize(parameters);
        robot.init(hardwareMap);

        robot.wobblyClaw.setPosition(-1);
        waitForStart();
//       bottomBlueSquareLS();
        //Shoot(1,8000);
        //sleep(50000);

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                if (tfod != null) {

                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if(updatedRecognitions.size() == 0){
                            telemetry.addData("Zero Run", 1);
                            ZeroLocation();

                        }
                        telemetry.update();

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        if(updatedRecognitions.size() != 0) {
                            for (Recognition recognition : updatedRecognitions) {
                                telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                                telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                        recognition.getLeft(), recognition.getTop());
                                telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                        recognition.getRight(), recognition.getBottom());

                                if (recognition.getLabel().equals("Quad")) {
                                    telemetry.addData("Zero Run", 2);
                                    QuadLocation();

                                    break;
                                } else if (recognition.getLabel().equals("Single")) {

                                    telemetry.addData("Zero Run", 3);
                                    SingleLocation();
                                }
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }

        if (tfod != null) {
            tfod.shutdown();
        }
    }

    /*
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "webcam");

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }


    public void ZeroLocation()
    {
        robot.wobblyClaw.setPosition(-1);
        DriveForwardDistance(1,48);
        StrafeLeftDistance(1,5);
        /*
        robot.wobblyClaw.setPosition(-1);
            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.wobblyJoint.setTargetPosition(robot.wobblyJoint.getCurrentPosition() + 600);
            robot.wobblyJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobblyJoint.setPower(-1);
            while (robot.wobblyJoint.isBusy()) {

            }
            robot.wobblyJoint.setPower(0);
            robot.wobblyClaw.setPosition(1);
         */

        StrafeRightDistance(1,24);
        TurnLeftDistance(.5,48);
        GyroCorrect(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle,180);
        /*
        SHOOT
         */
        StrafeLeftDistance(1,18);
        DriveForwardDistance(1,35);
        /*
            robot.wobblyClaw.setPosition(-1);
            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.wobblyJoint.setTargetPosition(robot.wobblyJoint.getCurrentPosition() - 600);
            robot.wobblyJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobblyJoint.setPower(1);
            while (robot.wobblyJoint.isBusy()) {

            }
            robot.wobblyJoint.setPower(0);
         */
        TurnRightDistance(.5,24);
        GyroCorrect(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, 90);
        StrafeRightDistance(1,35);
        DriveForwardDistance(1,20);
        /*
        robot.wobblyClaw.setPosition(-1);
            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.wobblyJoint.setTargetPosition(robot.wobblyJoint.getCurrentPosition() + 600);
            robot.wobblyJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobblyJoint.setPower(-1);
            while (robot.wobblyJoint.isBusy()) {

            }
            robot.wobblyJoint.setPower(0);
            robot.wobblyClaw.setPosition(1);
         */
        StrafeLeftDistance(1,10);

        sleep(5000);
        sleep(10000000);
    }
    //For Single Location, .45 is the power that is good for now
    public void SingleLocation()
    {
        robot.wobblyClaw.setPosition(-1);
        DriveForwardDistance(1,50);
        StrafeRightDistance(1,10);
        /*
        robot.wobblyClaw.setPosition(-1);
            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.wobblyJoint.setTargetPosition(robot.wobblyJoint.getCurrentPosition() + 600);
            robot.wobblyJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobblyJoint.setPower(-1);
            while (robot.wobblyJoint.isBusy()) {

            }
            robot.wobblyJoint.setPower(0);
            robot.wobblyClaw.setPosition(1);
        */
        DriveBackwardDistance(1,20);
        TurnLeftDistance(.5,48);
        GyroCorrect(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle,180);
        /*
        SHOOT
         */
        robot.intake.setPower(1);
        DriveBackwardDistance(1,30);
        robot.intake.setPower(0);
        DriveForwardDistance(1,30);
        /*
        SHOOT
        */
        StrafeLeftDistance(1,18);
        DriveForwardDistance(1,35);
        /*
            robot.wobblyClaw.setPosition(-1);
            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.wobblyJoint.setTargetPosition(robot.wobblyJoint.getCurrentPosition() - 600);
            robot.wobblyJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobblyJoint.setPower(1);
            while (robot.wobblyJoint.isBusy()) {

            }
            robot.wobblyJoint.setPower(0);
         */
        TurnRightDistance(.5,24);
        GyroCorrect(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, 90);
        StrafeRightDistance(1,40);
        DriveForwardDistance(1,15);
        /*
        robot.wobblyClaw.setPosition(-1);
            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.wobblyJoint.setTargetPosition(robot.wobblyJoint.getCurrentPosition() + 600);
            robot.wobblyJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobblyJoint.setPower(-1);
            while (robot.wobblyJoint.isBusy()) {

            }
            robot.wobblyJoint.setPower(0);
            robot.wobblyClaw.setPosition(1);
         */
        StrafeLeftDistance(1,15);
        sleep(100000000);
    }
    public void QuadLocation()
    {
        robot.wobblyClaw.setPosition(-1);
        DriveForwardDistance(1,70);
        StrafeLeftDistance(1,5);
        /*
        robot.wobblyClaw.setPosition(-1);
            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.wobblyJoint.setTargetPosition(robot.wobblyJoint.getCurrentPosition() + 600);
            robot.wobblyJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobblyJoint.setPower(-1);
            while (robot.wobblyJoint.isBusy()) {

            }
            robot.wobblyJoint.setPower(0);
            robot.wobblyClaw.setPosition(1);
         */

        StrafeRightDistance(1,24);
        TurnLeftDistance(.5,48);
        GyroCorrect(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle,180);
        DriveForwardDistance(1,30);
        /*
        SHOOT
         */
        StrafeLeftDistance(1,18);
        DriveForwardDistance(1,35);
        /*
            robot.wobblyClaw.setPosition(-1);
            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.wobblyJoint.setTargetPosition(robot.wobblyJoint.getCurrentPosition() - 600);
            robot.wobblyJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobblyJoint.setPower(1);
            while (robot.wobblyJoint.isBusy()) {

            }
            robot.wobblyJoint.setPower(0);
         */
        TurnRightDistance(.5,24);
        GyroCorrect(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle, 90);
        StrafeRightDistance(1,70);
        DriveForwardDistance(1,30);
        /*
        robot.wobblyClaw.setPosition(-1);
            robot.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            robot.wobblyJoint.setTargetPosition(robot.wobblyJoint.getCurrentPosition() + 600);
            robot.wobblyJoint.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.wobblyJoint.setPower(-1);
            while (robot.wobblyJoint.isBusy()) {

            }
            robot.wobblyJoint.setPower(0);
            robot.wobblyClaw.setPosition(1);
         */
        DriveBackwardDistance(1,20);
        StrafeLeftDistance(1,30);
    }
    public void DriveForwardDistance(double speed, double distanceInches)
    {
        double degree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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
        GyroCorrect(degree, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
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
        double degree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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
        GyroCorrect(degree, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        while(robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() )
        {

        }
        StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void TurnLeftDistance(double speed, double distanceInches)
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
    public void TurnRightDistance(double speed, double distanceInches)
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
    public void StrafeLeftDistance(double speed, double distanceInches)
    {
        double degree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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


        StrafeLeft(speed);
        GyroCorrect(degree, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        while(robot.rightDrive.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() && robot.leftDrive.isBusy()) {

        }

        StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
    public void StrafeRightDistance(double speed, double distanceInches)
    {
        double degree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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




        StrafeRight(speed);
        GyroCorrect(degree, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);

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
        double degree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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
        GyroCorrect(degree, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
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
        double degree = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
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
        GyroCorrect(degree, imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
        while(robot.leftDrive.isBusy() && robot.rightDrive.isBusy() && robot.leftBack.isBusy() && robot.rightBack.isBusy() )
        {

        }
        StopDriving();
        robot.leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.leftBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.rightBack.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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
    public void StrafeLeft(double power)
    {
        robot.leftDrive.setPower(-power);
        robot.rightDrive.setPower(power);
        robot.leftBack.setPower(power);
        robot.rightBack.setPower(-power);
    }
    public void StrafeRight(double power)
    {
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(-power);
        robot.leftBack.setPower(-power);
        robot.rightBack.setPower(power);
    }

    public void GyroCorrect(double degree1, double degree2){
        if(degree1 < degree2){
            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > degree1 + 0.5){
                robot.leftDrive.setPower(-.1);
                robot.rightDrive.setPower(.1);
                robot.leftBack.setPower(-.1);
                robot.rightBack.setPower(.1);
            }
        }
        else if(degree1 > degree2){
            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < degree1 - 0.5){
                robot.leftDrive.setPower(.1);
                robot.rightDrive.setPower(-.1);
                robot.leftBack.setPower(.1);
                robot.rightBack.setPower(-.1);
            }
        }
    }

    public void GyroFlip(double degree){
        if(degree > -180 && degree < -160){
            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > degree - 4){
                robot.leftDrive.setPower(.1);
                robot.rightDrive.setPower(-.1);
                robot.leftBack.setPower(-.1);
                robot.rightBack.setPower(.1);
            }
        }
        else if(degree < 180 && degree > 160){
            while(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < degree + 4){
                robot.leftDrive.setPower(-.1);
                robot.rightDrive.setPower(.1);
                robot.leftBack.setPower(.1);
                robot.rightBack.setPower(-.1);
            }
        }



    }
    public void Gyro180()
    {
        while (imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > -175) {
            newAngles = imu.getAngularOrientation(AxesReference.INTRINSIC,AxesOrder.ZYX,AngleUnit.DEGREES);

            if(imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle > -120){
                robot.leftDrive.setPower(-1);
                robot.rightDrive.setPower(1);
                robot.rightBack.setPower(1);
                robot.leftBack.setPower(-1);
            }
            else{
                robot.leftDrive.setPower(-0.2);
                robot.rightDrive.setPower(0.2);
                robot.rightBack.setPower(0.2);
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
        robot.rightDrive.setPower(-power);
        robot.rightBack.setPower(-power);
        robot.leftBack.setPower(-power);
        sleep(sleepTime);
        StopDriving();
    }
    public void LeftDiagonal(double power, int sleepTime)
    {
        robot.leftDrive.setPower(0);
        robot.rightDrive.setPower(-power);
        robot.rightBack.setPower(0);
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
        robot.leftBack.setPower(-power);
        robot.rightBack.setPower(-power);
        sleep(sleepTime);
        StopDriving();
    }
    public void StrafeLeft(double power, int sleepTime)
    {
        robot.leftDrive.setPower(power);
        robot.rightDrive.setPower(-power);
        robot.leftBack.setPower(-power);
        robot.rightBack.setPower(power);
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


}
