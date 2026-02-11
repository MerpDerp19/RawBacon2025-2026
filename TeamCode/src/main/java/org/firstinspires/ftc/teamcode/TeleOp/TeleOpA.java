package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

@TeleOp(name="TeleopA")
//@ServoType(flavor = ServoFlavor.CONTINUOUS)
//@DeviceProperties(xmlTag = "Servo", name = "@string/configTypeServo")

//comment
public class TeleOpA extends OpMode {


    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;
//    DcMotor armMotor;
//    Servo claw;
//    DcMotor ascentMotor;
//    Servo clawRotation;
//    DcMotor samplePivotMotor;
//    DcMotor sampleExtensionMotor;
//    Servo frictionStick;
    CRServo launcherTest;





    double CRServoPosition = 0;
    int PivotPosition = 0;
    int ExtensionPosition = 0;
    int PivotTarget = 0;
    int ExtensionTarget = 0;
    Double WheelSpeed;
    int ArmMotorPosition = 0;


    boolean Targeting = false;

    boolean Hanging = false;

    int Mode = 0;


    @Override
    public void init() {

        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");
 //       armMotor = hardwareMap.get(DcMotor.class, "armmotor");
 //       sampleExtensionMotor = hardwareMap.get(DcMotor.class, "extension");
        launcherTest = hardwareMap.get(CRServo.class, "launchertest");

//        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam1");


        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
        backright.setDirection(DcMotorSimple.Direction.REVERSE);
        frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
        backleft.setDirection(DcMotorSimple.Direction.FORWARD);

        WheelSpeed = 0.5;

//        armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//        armMotor.setDirection(DcMotorSimple.Direction.REVERSE);




    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

    }
    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (gamepad1.a)  launcherTest.setPower(1);

        if (gamepad1.x) launcherTest.setPower(0);


//        if (gamepad2.right_trigger == 1 && gamepad2.left_trigger == 1)
//            armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//
//
//        if (gamepad2.left_bumper)
//            Mode = 0;
//
//        if (gamepad2.right_bumper)
//            Mode = 1;
//
//
//
//        if (gamepad1.left_trigger == 1) {
//            WheelSpeed = 0.2;
//        } else if (gamepad1.left_trigger == 0) {
//            WheelSpeed = 0.7;
//        }
//
//        double Pad2LeftStickY = gamepad2.left_stick_y;
//        double Pad2RightStickY = -gamepad2.right_stick_y;
//        double LeftStickY = gamepad1.left_stick_y;
//        double LeftStickX = -gamepad1.left_stick_x;
//        double RightStickX = -gamepad1.right_stick_x;
//
//
//        frontright.setPower((-RightStickX / 1.5 + (LeftStickY - LeftStickX)) * WheelSpeed);
//        backright.setPower((-RightStickX / 1.5 + (LeftStickY + LeftStickX)) * WheelSpeed);
//        frontleft.setPower((RightStickX / 1.5 + (LeftStickY + LeftStickX)) * WheelSpeed);
//        backleft.setPower((RightStickX / 1.5 + (LeftStickY - LeftStickX)) * WheelSpeed);
//
//        if (Mode == 0) {
//            samplePivotMotor.setTargetPosition(PivotPosition);
//            sampleExtensionMotor.setTargetPosition(ExtensionPosition);
//            sampleExtensionMotor.setPower(0.3);
//            if (!Hanging) samplePivotMotor.setPower(0.3);
//        }
//
//
//        if (gamepad2.right_trigger == 1) {
//            if (Mode == 0) {
//                claw.setPosition(0);
//            } else {
//
//                frictionStick.setDirection(Servo.Direction.REVERSE);
//                frictionStick.setPosition(0.1); //inward
//
//            }
//
//
//        } else {
//            if (Mode == 0) {
//                claw.setPosition(0.35);
//            } else {
//
//                frictionStick.setDirection(Servo.Direction.FORWARD);
//                frictionStick.setPosition(0.1); //outward
//            }
//        }
//
//
//        clawRotation.setDirection(Servo.Direction.FORWARD);
//        clawRotation.setPosition(CRServoPosition);
//
//
//        if (gamepad2.dpad_right) {
//            if (clawRotation.getPosition() > 0.99 || clawRotation.getPosition() < 0.01) {
//                CRServoPosition += (0.010);
//            } else {
//                CRServoPosition += (0.007);
//
//            }
//        }
//        if (gamepad2.dpad_left) {
//            if (clawRotation.getPosition() > 0.99 || clawRotation.getPosition() < 0.01) {
//                CRServoPosition -= (0.007);
//            } else {
//                CRServoPosition -= (0.004);
//
//            }
//
//            if (CRServoPosition > 1)
//                CRServoPosition = 1;
//
//            if (CRServoPosition < 0)
//                CRServoPosition = 0;
//
//        }
//
//        //artemis was not here
//        // yes i was
//        //nuh uh
//
//        if (gamepad2.left_stick_y != 0) {
//            if (Mode == 0) {
//                Targeting = false;
//                armMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
//                armMotor.setPower(-gamepad2.left_stick_y / 2);
//            } else {
//
//            }
//        } else if (!Targeting)
//            armMotor.setPower(0);
//
//        if (gamepad2.a && Mode == 0) {
//            Targeting = true;
//            armMotor.setPower(0.6);
//            armMotor.setTargetPosition(240);
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//        if (gamepad2.y && Mode == 0) {
//            Targeting = true;
//            armMotor.setPower(0.6);
//            armMotor.setTargetPosition(1600);
//            armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        }
//
//
//        if (Mode == 1) {
//
//            sampleExtensionMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            sampleExtensionMotor.setPower(0.5);
//            sampleExtensionMotor.setTargetPosition(ExtensionTarget);
//
//            ExtensionTarget += gamepad2.left_stick_y * 14;
//
//            if (ExtensionTarget > 0)
//                ExtensionTarget = 0;
//
//            if (ExtensionTarget < -2000)
//                ExtensionTarget = -2000;
//
//
//            samplePivotMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            if (!Hanging) samplePivotMotor.setPower(0.5);
//            if (PivotTarget < -300 && !Hanging)
//                samplePivotMotor.setPower(0.5 + (ExtensionTarget / 3200));
//            samplePivotMotor.setTargetPosition(PivotTarget);
//
//            PivotTarget += -gamepad2.right_stick_y * 4;
//
//            if (PivotPosition < -400 && ExtensionPosition > 400) {
//                if (PivotTarget > -400)
//                    PivotTarget = -400;
//            }
//
//        }
//
//
//        ArmMotorPosition = armMotor.getCurrentPosition();
//
//        PivotPosition = samplePivotMotor.getCurrentPosition();
//
//        ExtensionPosition = sampleExtensionMotor.getCurrentPosition();


        telemetry.addData("ArmMotorPosition: ", ArmMotorPosition);
        telemetry.addData("PivotMotorPostion: ", PivotPosition);
        telemetry.addData("ExtensionPosition: ", ExtensionPosition);
        telemetry.addData("ExtensionTarget: ", ExtensionTarget);


        telemetry.update();

    }
    @Override
    public void stop(){


    }
}