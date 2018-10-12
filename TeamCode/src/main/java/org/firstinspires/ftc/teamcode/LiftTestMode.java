package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp(name="LiftTestMode", group="Iterative Opmode")
public class LiftTestMode extends OpMode{

    private DcMotor liftMotor = null;
    private DigitalChannel digitalChannelUp = null;
    private DigitalChannel digitalChannelDown = null;
    private boolean liftPowered = false;


    @Override
    public void init() {
        liftMotor = hardwareMap.get(DcMotor.class,"LiftMotor");
        digitalChannelUp = hardwareMap.get(DigitalChannel.class,"DigitalChannelUp");
        digitalChannelDown = hardwareMap.get(DigitalChannel.class, "DigitalChannelDown");

        digitalChannelUp.setMode(DigitalChannel.Mode.INPUT);
        digitalChannelDown.setMode(DigitalChannel.Mode.INPUT);

        liftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        if(!digitalChannelDown.getState()){
            liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        telemetry.addData("DigitalChannelDown",digitalChannelDown.getState());

    }

    @Override
    public void loop() {
        if(gamepad2.b){
            liftPowered = true;
        }

        if (liftPowered) {
            liftMotor.setPower(1);
        }

        if (!digitalChannelUp.getState() || !digitalChannelDown.getState()){
            liftMotor.setPower(0);
            reverseDirection();
            liftPowered = false;
        }
    }

    private void reverseDirection() {
        if (liftMotor.getDirection() == DcMotorSimple.Direction.FORWARD) {
            liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            liftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        }
    }
}
