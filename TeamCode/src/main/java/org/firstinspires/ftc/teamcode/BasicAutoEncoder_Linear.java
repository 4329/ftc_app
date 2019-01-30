/* Copyright (c) 2017 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/**
 * This file illustrates the concept of driving a path based on encoder counts.
 * It uses the common Pushbot hardware class to define the drive on the robot.
 * The code is structured as a LinearOpMode
 * <p>
 * The code REQUIRES that you DO have encoders on the wheels,
 * otherwise you would use: PushbotAutoDriveByTime;
 * <p>
 * This code ALSO requires that the drive Motors have been configured such that a positive
 * power command moves them forwards, and causes the encoders to count UP.
 * <p>
 * The desired path in this example is:
 * - Drive forward for 48 inches
 * - Spin right for 12 Inches
 * - Drive Backwards for 24 inches
 * - Stop and close the claw.
 * <p>
 * The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 * that performs the actual movement.
 * This methods assumes that each movement is relative to the last stopping place.
 * There are other ways to perform encoder based moves, but this method is probably the simplest.
 * This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 * <p>
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@Autonomous(name = "AutoWithLift", group = "Cadets")

public class BasicAutoEncoder_Linear extends LinearOpMode {

    /* Declare OpMode members. */
    CadetHardware robot = new CadetHardware();   // Use a Pushbot's hardware
    private ElapsedTime runtime = new ElapsedTime();

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";
    private static final String VUFORIA_KEY = "AYdWLp7/////AAABmfV9D1yBc0pOi1LU+BKWlvVworZmGrQJO7RDfUf/fzFdCPB5pPyZHvbcCizQa/wciSGPG+pAY3E6yXq/aXQD7yIlsSWo8CYs5QlH7sGfzfmy//rdtToFlMmUxP7Mf/obD/90x9CODqEeOYgZJfsrO1B5GYKbMpwsxmsuMBkFkDCjvTyNhavldYXXayHvobP/cS6x91EGVDzaKEFhJWEL4vJR2XvUhBSQdDORu/AGlE/vSu098NUW/3kvUANyvOLMhbMdpM7zTJXcakgHVy8CLrJ2R1ZSi34efaI5plCrwAUL6E2Og0Qqk5cjNckfksFp0cQb6RU0HnDpwdYf3lt9tHNicvb+w3KPL9GXV1vibbF8";
    private VuforiaLocalizer vuforia;
    private TFObjectDetector tfod;
    private int detectedGoldMineralX = -1;
    private String detectedGoldPosition = null;
    private double detectedGoldAngle;

    static final double COUNTS_PER_MOTOR_REV = 1120;    // eg: NeveRest Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.286;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double SLOW_SPEED = 0.2;
    static final double HIGH_SPEED = 0.8;
    static final double DRIVE_SPEED = 0.6;
    static final double TURN_SPEED = 0.4;
    static final double CRATER = -25;
    static long DETECT_GOLD_DELAY = 3000;

    private int autoStartDelay = 0;
    private boolean isDepot = true;

    @Override
    public void runOpMode() {

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.init(hardwareMap);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");    //
        telemetry.update();

        encoderMode();

        // Send telemetry message to indicate successful Encoder reset
        telemetry.addData("Path0", "Starting at %7d :%7d",
                robot.frontLeftDrive.getCurrentPosition(),
                robot.frontRightDrive.getCurrentPosition(),
                robot.frontLeftDrive.getCurrentPosition(),
                robot.frontRightDrive.getCurrentPosition());
        telemetry.update();

        //imu gyro calibration
        while (!isStopRequested() && !robot.imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "Waiting for Start");
        telemetry.addData("Imu Calib Status", robot.imu.getCalibrationStatus().toString());

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        sleep(autoStartDelay * 1000);

        detectGoldInit();
        detectGold();


        robot.liftMotor.setPower(CadetConstants.LIFT_MOTOR_POWER_DOWN);
//        robot.backLeftDrive.setPower(-SLOW_SPEED);
//        robot.frontLeftDrive.setPower(-SLOW_SPEED);

        while (opModeIsActive()) {
            if (!robot.digitalChannelUp.getState()) {
                robot.liftMotor.setPower(0);
//                robot.backLeftDrive.setPower(0);
//                robot.backRightDrive.setPower(0);
                break;
            }
            idle();
        }
        robot.liftServo.setPosition(0.58);

//        encoderDrive(DRIVE_SPEED, -12, 12, 10.0);
//        encoderDrive(SLOW_SPEED, 3, 3, 10.0);

        // Open scoop to push element into depot/get out of crater wall way
        robot.scoopServo.setPosition(0);

        sleep(1000);

        if (detectedGoldPosition == null){
            detectGold();
            if (detectedGoldPosition == null){
                detectedGoldPosition = "Center";
            }
        }


        //Drive back to turn
        encoderDrive(DRIVE_SPEED, -2, -2, 10.0);


//        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        //imu 137 - Alpha, 143 - Bravo
//        while (opModeIsActive() &&
//                robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle < 137){
//            robot.frontLeftDrive.setPower(-TURN_SPEED);
//            robot.backLeftDrive.setPower(-TURN_SPEED);
//            robot.frontRightDrive.setPower(TURN_SPEED);
//            robot.backRightDrive.setPower(TURN_SPEED);
//            idle();
//        }


        knockOffGold();

        detectGoldShutdown();

        if (isDepot){
            doDepot();
        }
        else {
            doCrater();
        }



        telemetry.addData("Path", "Complete");
        telemetry.update();
    }



    private void detectGold() {
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            long now = System.currentTimeMillis();

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (System.currentTimeMillis() > now + DETECT_GOLD_DELAY){
                            detectedGoldPosition = null;
                            break;
                        }
                        if (updatedRecognitions.size() == 2) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            //Not in use, keeping to stay consistent with sample
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {

                                    detectedGoldAngle = recognition.estimateAngleToObject(AngleUnit.DEGREES);

                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                                telemetry.addData("EstimateAngle", recognition.estimateAngleToObject(AngleUnit.DEGREES));
                                telemetry.addData("goldMineralX", goldMineralX);
                            }
                            if (goldMineralX != -1) {
                                if (goldMineralX < silverMineral1X) {
                                    detectedGoldPosition = "Center";
                                } else {
                                    detectedGoldPosition = "Right";
                                }

                                telemetry.addData("Gold Mineral Position", detectedGoldPosition);
                                detectedGoldMineralX = goldMineralX;
                                telemetry.update();

                            }
                            else {
                                detectedGoldPosition = "Left";
                            }
                            break;
                        }
                        telemetry.update();
                    }
                }
            }

        }
    }

    private void detectGoldShutdown() {
        if (tfod != null) {
            tfod.shutdown();
        }
    }

    private void detectGoldInit() {
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }
    }


    private void knockOffGold() {
        encoderMode();
        if ( detectedGoldPosition.equals("Center")){
            encoderDrive(DRIVE_SPEED, -12, -12, 10.0);
        }
        if (detectedGoldPosition.equals("Right")){
            encoderDrive(TURN_SPEED, 3, -3, 10.0);
            encoderDrive(DRIVE_SPEED, -22, -22, 10.0);
        }
        if (detectedGoldPosition.equals("Left")){
            encoderDrive(TURN_SPEED, -3, 3, 10.0);
            encoderDrive(DRIVE_SPEED, -22, -22, 10.0);
        }
    }

    private void doDepot() {
        encoderMode();
        if ( detectedGoldPosition.equals("Center")){
            encoderDrive(DRIVE_SPEED, -26, -26, 10.0);
            sleep(500);
            robot.markerServo.setPosition(-1);
            sleep(500);
            encoderDrive(TURN_SPEED, 5, -5, 10.0);
            encoderDrive(HIGH_SPEED, 35, 35, 10.0);
            encoderDrive(DRIVE_SPEED, -1, 1, 10.0);
            encoderDrive(HIGH_SPEED, 16, 16, 10.0);
        }
        if (detectedGoldPosition.equals("Right")){
            encoderDrive(DRIVE_SPEED, -7, -7, 10.0);
            encoderDrive(DRIVE_SPEED, -7, 7, 10.0);
            encoderDrive(DRIVE_SPEED, -15, -15, 10.0);
            sleep(500);
            robot.markerServo.setPosition(-1);
            sleep(500);
            encoderDrive(HIGH_SPEED, 35, 35, 10.0);
            encoderDrive(DRIVE_SPEED, -1.5, 1.5, 10.0);
            encoderDrive(HIGH_SPEED, 12, 12, 10.0);
        }
        if (detectedGoldPosition.equals("Left")){
            encoderDrive(DRIVE_SPEED, -7, -7, 10.0);
            encoderDrive(DRIVE_SPEED, 7, -7, 10.0);
            encoderDrive(DRIVE_SPEED, -15, -15, 10.0);
            sleep(500);
            robot.markerServo.setPosition(-1);
            sleep(500);
            encoderDrive(HIGH_SPEED, 35, 35, 10.0);
            encoderDrive(DRIVE_SPEED, -1.5, 1.5, 10.0);
            encoderDrive(HIGH_SPEED, 12, 12, 10.0);
        }
    }

    private void doCrater() {
        encoderMode();
        if ( detectedGoldPosition.equals("Center")){
            encoderDrive(DRIVE_SPEED, -14, -14, 10.0);
        }
        if (detectedGoldPosition.equals("Right")){
            encoderDrive(DRIVE_SPEED, -18, -18, 10.0);
        }
        if (detectedGoldPosition.equals("Left")){
            encoderDrive(DRIVE_SPEED, -18, -18, 10.0);
        }
    }


    private void encoderMode() {
        robot.frontLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

//    private void doDepot() {
//        encoderDrive(DRIVE_SPEED, DEPOT, DEPOT, 10.0);
//        robot.markerServo.setPosition(1);
//        encoderDrive(TURN_SPEED, 5, -5, 10.0);
//        sleep(500);
//        encoderDrive(DRIVE_SPEED, 35, 35, 10.0);
//        encoderDrive(DRIVE_SPEED, -1, 1, 10.0);
//        encoderDrive(DRIVE_SPEED, 16, 16, 10.0);
//    }

    @Override
    public synchronized void waitForStart() {
        while (!isStarted()) {
            synchronized (this) {
                try {
                    if (gamepad1.a){
                        autoStartDelay = 3;
                    }
                    if (gamepad1.b){
                        autoStartDelay = 5;
                    }
                    if (gamepad1.y){
                        autoStartDelay = 0;
                    }
                    if (gamepad1.dpad_up){
                        isDepot = true;
                    }
                    if (gamepad1.dpad_down){
                        isDepot = false;
                    }

                    telemetry.addData("Start Delay", autoStartDelay + " Y = 0, A = 3, B = 5");
                    telemetry.addData("isDepot", isDepot + " Dpad Up = Depot, Dpad Down = Crater");
                    telemetry.addData("Imu", robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle);
                    telemetry.update();
                    this.wait();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                    return;
                }
            }
        }
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the opmode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = robot.frontLeftDrive.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = robot.frontRightDrive.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            robot.frontLeftDrive.setTargetPosition(newLeftTarget);
            robot.frontRightDrive.setTargetPosition(newRightTarget);
            robot.backLeftDrive.setTargetPosition(newLeftTarget);
            robot.backRightDrive.setTargetPosition(newRightTarget);

            // Turn On RUN_TO_POSITION
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            robot.frontLeftDrive.setPower(Math.abs(speed));
            robot.frontRightDrive.setPower(Math.abs(speed));
            robot.backLeftDrive.setPower(Math.abs(speed));
            robot.backRightDrive.setPower(Math.abs(speed));

            // keep looping while we are still active, and there is time left, and both motors are running.
            // Note: We use (isBusy() && isBusy()) in the loop test, which means that when EITHER motor hits
            // its target position, the motion will stop.  This is "safer" in the event that the robot will
            // always end the motion as soon as possible.
            // However, if you require that BOTH motors have finished their moves before the robot continues
            // onto the next step, use (isBusy() || isBusy()) in the loop test.
            while (opModeIsActive() &&
                    (runtime.seconds() < timeoutS) &&
                    (robot.frontLeftDrive.isBusy() && robot.frontRightDrive.isBusy()) &&
                    (robot.backLeftDrive.isBusy() && robot.backRightDrive.isBusy())) {


                // Display it for the driver.
                telemetry.addData("Path1", "Running to %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Path2", "Running at %7d :%7d",
                        robot.frontLeftDrive.getCurrentPosition(),
                        robot.frontRightDrive.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            robot.frontLeftDrive.setPower(0);
            robot.frontRightDrive.setPower(0);
            robot.backLeftDrive.setPower(0);
            robot.backRightDrive.setPower(0);

            // Turn off RUN_TO_POSITION
            robot.frontLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.frontRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backLeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            robot.backRightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            //  sleep(250);   // optional pause after each move
        }
    }

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the Tensor Flow Object Detection engine.
    }

    /**
     * Initialize the Tensor Flow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);
    }

}
