package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Drive implements SubOpMode {

    private Telemetry telemetry;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private DcMotor leftDrive = null;
    private DcMotor rightDrive = null;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {
        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        telemetry.addData("Status", "Drive Initialized Start");
        leftDrive  = hardwareMap.get(DcMotor.class, "FrontLeftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "FrontRightDrive");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);

        // Tell the driver that initialization is complete.
        telemetry.addData("Status", "Drive Initialized Complete");
    }

    @Override
    public void loop() {
        double leftPower;
        double rightPower;

        leftPower  = -gamepad1.left_stick_y ;
        rightPower = -gamepad1.right_stick_y ;

        // Send calculated power to wheels
        leftDrive.setPower(leftPower);
        rightDrive.setPower(rightPower);

        // Show the elapsed game time and wheel power.
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
    }
}
