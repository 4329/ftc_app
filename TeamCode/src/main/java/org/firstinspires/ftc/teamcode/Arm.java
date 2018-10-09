package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

class Arm implements SubOpMode {

    private Telemetry telemetry;
    private Gamepad gamepad1;
    private Gamepad gamepad2;
    private Servo armServo;
    private int count;

    @Override
    public void init(HardwareMap hardwareMap, Telemetry telemetry, Gamepad gamepad1, Gamepad gamepad2) {

        this.telemetry = telemetry;
        this.gamepad1 = gamepad1;
        this.gamepad2 = gamepad2;

        telemetry.addData("Status", "Arm Initialized Start");
        armServo = hardwareMap.get(Servo.class, "ArmServo");
        telemetry.addData("Status", "Arm Initialized Complete");

    }

    @Override
    public void loop() {
        count++;
        if (armServo.getPosition() == 1 && (count % 100 == 0)) {
            armServo.setPosition(0);
        } else {
            armServo.setPosition(1);
        }
        telemetry.addData("arm", "Servo " + armServo.getPosition());
        telemetry.update();
    }
}
