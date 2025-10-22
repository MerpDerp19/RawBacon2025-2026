package org.firstinspires.ftc.teamcode.TeleOp;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="CatapultTesting")
//@ServoType(flavor = ServoFlavor.CONTINUOUS)
//@DeviceProperties(xmlTag = "Servo", name = "@string/configTypeServo")

//comment
public class CatapultTesting extends OpMode {


//    DcMotor frontleft;
//    DcMotor frontright;
//    DcMotor backleft;
//    DcMotor backright;
//    CRServo leftintake;
//    CRServo rightintake;
    DcMotor spinner;






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
    int SpinnerTarget = 0;

    @Override
    public void init() {

//        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
//        frontright = hardwareMap.get(DcMotor.class, "frontright");
//        backleft = hardwareMap.get(DcMotor.class, "backleft");
//        backright = hardwareMap.get(DcMotor.class, "backright");
//        leftintake = hardwareMap.get(CRServo.class, "leftintake");
//        rightintake = hardwareMap.get(CRServo.class, "rightintake");
        spinner = hardwareMap.get(DcMotor.class, "spinner");


        // ascentMotor = hardwareMap.get(DcMotor.class, "ascentmotor");
        // clawRotation = hardwareMap.get(Servo.class, "clawrotation");
        // frictionStick = hardwareMap.get(Servo.class, "frictionstick");
        //  samplePivotMotor = hardwareMap.get(DcMotor.class, "pivot");

//        frontright.setDirection(DcMotorSimple.Direction.REVERSE);
//        backright.setDirection(DcMotorSimple.Direction.REVERSE);
//        frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
//        backleft.setDirection(DcMotorSimple.Direction.FORWARD);

        WheelSpeed = 0.5;


        //   samplePivotMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //   sampleExtensionMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



    }


    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        spinner.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

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

        if (gamepad2.left_bumper)
            Mode = 0;

        if (gamepad2.right_bumper)
            Mode = 1;



        if (gamepad1.left_trigger == 1) {
            WheelSpeed = 0.2;
        } else if (gamepad1.left_trigger == 0) {
            WheelSpeed = 0.7;
        }

//        if (gamepad1.right_trigger == 1) {
//            leftintake.setPower(0.7);
//            rightintake.setPower(-0.7);
//        } else if (gamepad1.right_trigger == 0) {
//            leftintake.setPower(0);
//            rightintake.setPower(0);
//        }

        if (spinner.getCurrentPosition() >= SpinnerTarget) {
            SpinnerTarget += 1460;
        } else if (gamepad1.a) {

            spinner.setPower(0.5);
            spinner.setTargetPosition(SpinnerTarget);
            spinner.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        }

        double Pad2LeftStickY = gamepad2.left_stick_y;
        double Pad2RightStickY = -gamepad2.right_stick_y;
        double LeftStickY = gamepad1.left_stick_y;
        double LeftStickX = -gamepad1.left_stick_x;
        double RightStickX = -gamepad1.right_stick_x;


//        frontright.setPower((-RightStickX / 1.5 + (LeftStickY - LeftStickX)) * WheelSpeed);
//        backright.setPower((-RightStickX / 1.5 + (LeftStickY + LeftStickX)) * WheelSpeed);
//        frontleft.setPower((RightStickX / 1.5 + (LeftStickY + LeftStickX)) * WheelSpeed);
//        backleft.setPower((RightStickX / 1.5 + (LeftStickY - LeftStickX)) * WheelSpeed);

//        if (Mode == 0) {
//            samplePivotMotor.setTargetPosition(PivotPosition);
//            sampleExtensionMotor.setTargetPosition(ExtensionPosition);
//            sampleExtensionMotor.setPower(0.3);
//            if (!Hanging) samplePivotMotor.setPower(0.3);
//        }





//        PivotPosition = samplePivotMotor.getCurrentPosition();
//
//        ExtensionPosition = sampleExtensionMotor.getCurrentPosition();


//        telemetry.addData("ArmMotorPosition: ", ArmMotorPosition);
//        telemetry.addData("PivotMotorPostion: ", PivotPosition);
//        telemetry.addData("ExtensionPosition: ", ExtensionPosition);
//        telemetry.addData("ExtensionTarget: ", ExtensionTarget);
        telemetry.addData("spinnerpos: ", spinner.getCurrentPosition());
        telemetry.addData("SpinnerTarget: ", SpinnerTarget);

//        telemetry.addData("LeftPower: ", leftintake.getPower());
//        telemetry.addData("RightPower: ", rightintake.getPower());



        telemetry.update();

    }
    @Override
    public void stop(){


    }
}