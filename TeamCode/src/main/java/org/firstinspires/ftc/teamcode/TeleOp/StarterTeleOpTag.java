/*
 * Copyright (c) 2025 FIRST
 * All rights reserved.
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
 * Neither the name of FIRST nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.TeleOp;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Utility.*;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.teamcode.Utility.ColorSensor;

import java.util.List;


/*
 * This file includes a teleop (driver-controlled) file for the goBILDA® StarterBot for the
 * 2025-2026 FIRST® Tech Challenge season DECODE™. It leverages a differential/Skid-Steer
 * system for robot mobility, one high-speed motor driving two "launcher wheels", and two servos
 * which feed that launcher.
 *
 * Likely the most niche concept we'll use in this example is closed-loop motor velocity control.
 * This control method reads the current speed as reported by the motor's encoder and applies a varying
 * amount of power to reach, and then hold a target velocity. The FTC SDK calls this control method
 * "RUN_USING_ENCODER". This contrasts to the default "RUN_WITHOUT_ENCODER" where you control the power
 * applied to the motor directly.
 * Since the dynamics of a launcher wheel system varies greatly from those of most other FTC mechanisms,
 * we will also need to adjust the "PIDF" coefficients with some that are a better fit for our application.
 */

@TeleOp(name = "StarterBotTeleopTag", group = "StarterBot")
//@Disabled
public class StarterTeleOpTag extends OpMode {
    final double FEED_TIME_SECONDS = 0.15; //The feeder servos run this long when a shot is requested.
    final double LAUNCHER_TIME_SECONDS = 0;
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    final double ROTATE_CONSTANT = 0.016;
    final double X_CONSTANT = -0.1;
    final double Y_CONSTANT = -0.1;
    double ROT_OFFSET = 8;
    final double X_OFFSET = 5;
    final double Y_OFFSET = 50;

    /*
     * When we control our launcher motor, we are using encoders. These allow the control system
     * to read the current speed of the motor and apply more or less power to keep it at a constant
     * velocity. Here we are setting the target, and minimum velocity that the launcher should run
     * at. The minimum velocity is a threshold for determining when to fire.
     */
    final double LAUNCHER_TARGET_VELOCITY = 1125;
    final double LAUNCHER_MIN_VELOCITY = 475;
    double yawTarget = 0;
    double xTarget = 0;
    boolean tagSpotted = false;

    double angRate = 0;



    // Declare OpMode members.
//    private DcMotor leftDrive = null;
//    private DcMotor rightDrive = null;
    DcMotor frontleft;
    DcMotor frontright;
    DcMotor backleft;
    DcMotor backright;

    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;

    Servo cam;

    NormalizedColorSensor sensor;

    ElapsedTime feederTimer = new ElapsedTime();

    ElapsedTime launcherTimer = new ElapsedTime();

    double disI = 0;

    double camAngle = 75;

    double camPos = 1;

    double disM = 0;

    // 1.35 - 1.60 magic number
    double MagicNumber = 1.35;

    // april tag coods for far launch
    // x = 7.27
    //y = 108.6
    // z = -8.94
    //yaw = -30.45


    /*
     * TECH TIP: State Machines
     * We use a "state machine" to control our launcher motor and feeder servos in this program.
     * The first step of a state machine is creating an enum that captures the different "states"
     * that our code can be in.
     * The core advantage of a state machine is that it allows us to continue to loop through all
     * of our code while only running specific code when it's necessary. We can continuously check
     * what "State" our machine is in, run the associated code, and when we are done with that step
     * move on to the next state.
     * This enum is called the "LaunchState". It reflects the current condition of the shooter
     * motor and we move through the enum when the user asks our code to fire a shot.
     * It starts at idle, when the user requests a launch, we enter SPIN_UP where we get the
     * motor up to speed, once it meets a minimum speed then it starts and then ends the launch process.
     * We can use higher level code to cycle through these states. But this allows us to write
     * functions and autonomous routines in a way that avoids loops within loops, and "waits".
     */
    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
    }
    boolean isOnBlueTeam = false;
    private enum TagAlignState {
        NO_TAG,
        ROTATE,
        X_POS,
        Y_POS,
        DONE,
    }


    private LaunchState launchState;
    private TagAlignState tagAlignState;
    private void rotateByAmount(double z) {
        /*
        if (z > 0) {
            frontleft.setDirection(DcMotorSimple.Direction.FORWARD);
            frontright.setDirection(DcMotorSimple.Direction.FORWARD);
            backleft.setDirection(DcMotorSimple.Direction.FORWARD);
            backright.setDirection(DcMotorSimple.Direction.FORWARD);
        } else {
            frontleft.setDirection(DcMotorSimple.Direction.REVERSE);
            frontright.setDirection(DcMotorSimple.Direction.REVERSE);
            backleft.setDirection(DcMotorSimple.Direction.REVERSE);
            backright.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        */
        frontleft.setPower(z*ROTATE_CONSTANT * 1);
        frontright.setPower(z*ROTATE_CONSTANT * -1);
        backleft.setPower(z*ROTATE_CONSTANT * 1);
        backright.setPower(z*ROTATE_CONSTANT * -1);
    }
    private void xByAmount(double x) {
        frontleft.setPower(x*X_CONSTANT * 1);
        frontright.setPower(x*X_CONSTANT * -1);
        backleft.setPower(x*X_CONSTANT * -1);
        backright.setPower(x*X_CONSTANT * 1);
    }
    private void yByAmount(double y) {
        frontleft.setPower(y*Y_CONSTANT * 1);
        frontright.setPower(y*Y_CONSTANT * 1);
        backleft.setPower(y*Y_CONSTANT * 1);
        backright.setPower(y*Y_CONSTANT * 1);
    }

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftPower;
    double rightPower;

    private AprilTagVision vision = new AprilTagVision();
    private ColorSensor sensing = new ColorSensor();

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        launchState = LaunchState.IDLE;
        tagAlignState = TagAlignState.NO_TAG;

        vision.init(hardwareMap);

        /*
         * Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step.
         */
//        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
//        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");
        frontleft = hardwareMap.get(DcMotor.class, "frontleft");
        frontright = hardwareMap.get(DcMotor.class, "frontright");
        backleft = hardwareMap.get(DcMotor.class, "backleft");
        backright = hardwareMap.get(DcMotor.class, "backright");

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo.class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        sensor = hardwareMap.get(NormalizedColorSensor.class, "color_sensor");
        cam = hardwareMap.get(Servo.class, "cam");


        /*
         * To drive forward, most robots need the motor on one side to be reversed,
         * because the axles point in opposite directions. Pushing the left stick forward
         * MUST make robot go forward. So adjust these two lines based on your first test drive.
         * Note: The settings here assume direct drive on left and right wheels. Gear
         * Reduction or 90 Deg drives may require direction flips
         */
//        leftDrive.setDirection(DcMotor.Direction.REVERSE);
//        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        /*
         * Here we set our launcher to the RUN_USING_ENCODER runmode.
         * If you notice that you have no control over the velocity of the motor, it just jumps
         * right to a number much higher than your set point, make sure that your encoders are plugged
         * into the port right beside the motor itself. And that the motors polarity is consistent
         * through any wiring.
         */
        //   launcher.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /*
         * Setting zeroPowerBehavior to BRAKE enables a "brake mode". This causes the motor to
         * slow down much faster when it is coasting. This creates a much more controllable
         * drivetrain. As the robot stops much quicker.
         */
//        leftDrive.setZeroPowerBehavior(BRAKE);
//        rightDrive.setZeroPowerBehavior(BRAKE);
        launcher.setZeroPowerBehavior(BRAKE);

        /*
         * set Feeders to an initial value to initialize the servo controller
         */
        leftFeeder.setPower(STOP_SPEED);
        rightFeeder.setPower(STOP_SPEED);

        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        launcher.setDirection(DcMotorSimple.Direction.FORWARD);

        backright.setDirection(DcMotorSimple.Direction.REVERSE);
        frontright.setDirection(DcMotorSimple.Direction.REVERSE);




        /*
         * Much like our drivetrain motors, we set the left feeder servo to reverse so that they
         * both work to feed the ball into the robot.
         */
        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        /*
         * Tell the driver that initialization is complete.
         */
        telemetry.addData("Status", "Initialized");


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit START
     */
    @Override
    public void init_loop() {
        if (gamepad2.dpad_left) {
            isOnBlueTeam=true;
        }
        if (gamepad2.dpad_right) {
            isOnBlueTeam=false;
        }

        ColorSensor.DetectedColor color = sensing.getDetectedColor(sensor);

        telemetry.addData("red: ", sensing.normRed);
        telemetry.addData("green: ", sensing.normGreen);
        telemetry.addData("blue: ", sensing.normBlue);

        if (color == ColorSensor.DetectedColor.BLUE)
            isOnBlueTeam = true;
         else
            isOnBlueTeam = false;
    }

    /*
     * Code to run ONCE when the driver hits START
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits START but before they hit STOP
     */
    @Override
    public void loop() {
        /*
         * Here we call a function called arcadeDrive. The arcadeDrive function takes the input from
         * the joysticks, and applies power to the left and right drive motor to move the robot
         * as requested by the driver. "arcade" refers to the control style we're using here.
         * Much like a classic arcade game, when you move the left joystick forward both motors
         * work to drive the robot forward, and when you move the right joystick left and right
         * both motors work to rotate the robot. Combinations of these inputs can be used to create
         * more complex maneuvers.
         */
        //arcadeDrive(-gamepad1.left_stick_y, gamepad1.right_stick_x);

        AprilTagDetection tag = null;
        //april tag vision
        List<AprilTagDetection> detections = vision.getDetections();
        if (!detections.isEmpty()) {
            tag = detections.get(0);
            telemetry.addData("Tag ID", tag.id);
            if (tag.ftcPose != null) {
                telemetry.addData("x", tag.ftcPose.x);
                telemetry.addData("y", tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("yaw ", tag.ftcPose.yaw);
                tagSpotted = true;

            }
        } else {
            telemetry.addLine("No tags detected");
            tagSpotted = false;
        }





        //sets drivetrain
        if (gamepad1.left_bumper){
            //slow mode
            omniwheelDrive(0.3);
        }
        else if (gamepad1.right_bumper){
            //fast mode
            omniwheelDrive(1);
        }
        //normal
        else omniwheelDrive(0.75);



        if (tagSpotted && gamepad2.x) {

            disI = Math.hypot(tag.ftcPose.x, tag.ftcPose.y);

            //converts from inches to meters
            disM = (disI * 2.54) / 100;

            //linear functions for cam angle
            //camAngle is what is plugged into getLaunchValue (degrees)
            //camPos is the position we set the servo to
//            camAngle = -0.106195 * disI + 80;
//            camPos = -0.00446429 * disI + 1;

            MagicNumber = 0.00378788 * disI +1.17197;

            camAngle = 68;
            camPos = 0.5;


            //inputs distance from goal (meters), angle of launch (radians), and change in height (meters)
            angRate = getLaunchValue(disM, Math.toRadians(camAngle) , 0.35);


            // 1 = 80 degrees
            // 0.5 = 68 degrees

        }

        // sets cam position and limits between 0.5 and 1
        cam.setPosition(Math.max(0.5, Math.min(camPos, 1)));


        telemetry.addData("distance (inches): ", disI);
        telemetry.addData("distance (meters): ", disM);
        telemetry.addData("camAngle: ", camAngle);
        telemetry.addData("camPos: ", camPos);



        //against goal preset
        if (gamepad2.a){
            angRate = 140;
            camPos = 1;
        }



        if (gamepad2.y) {
            //sets velocity in degrees per second, 360/19.23 = 1 rev/sec
            //we divide by 19.23 because we changed the gear ratio of the launch motor
            launcher.setVelocity(angRate, AngleUnit.DEGREES);

        } else if (gamepad2.b) { // stop flywheel
            launcher.setVelocity(STOP_SPEED);

        }


        if (gamepad2.dpad_left) {
            isOnBlueTeam=true;
            ROT_OFFSET = 8;
        }
        if (gamepad2.dpad_right) {
            isOnBlueTeam=false;
            ROT_OFFSET = -8;
        }



        if (gamepad1.x) {
            if (tagSpotted) {
                switch (tagAlignState) {
                    case NO_TAG:
                        if (((((tag.id - 20) / 4) == 1) ^ isOnBlueTeam) == true) {
                            tagAlignState = TagAlignState.ROTATE;
                        }
                        break;
                    case ROTATE:
                        rotateByAmount(tag.ftcPose.yaw - ROT_OFFSET);
                        if (Math.abs(tag.ftcPose.yaw - ROT_OFFSET) < 3) {tagAlignState = TagAlignState.X_POS;}
                        break;
                    case X_POS:
                        xByAmount(tag.ftcPose.x - X_OFFSET);
                        if (Math.abs(tag.ftcPose.x) < 4) {tagAlignState = TagAlignState.Y_POS;}
                        if (Math.abs(tag.ftcPose.yaw - ROT_OFFSET) > 3) {tagAlignState = TagAlignState.ROTATE;}
                        break;
                    case Y_POS:
                        yByAmount(tag.ftcPose.y - Y_OFFSET);
                        if (Math.abs(tag.ftcPose.y - Y_OFFSET) < 3) {tagAlignState = TagAlignState.DONE;}
                        if (Math.abs(tag.ftcPose.x - X_OFFSET) > 4) {tagAlignState = TagAlignState.X_POS;}
                        if (Math.abs(tag.ftcPose.yaw - ROT_OFFSET) > 3) {tagAlignState = TagAlignState.ROTATE;}
                        break;
                    case DONE:
                        if (Math.abs(tag.ftcPose.y - Y_OFFSET) > 3) {tagAlignState = TagAlignState.Y_POS;}
                        if (Math.abs(tag.ftcPose.x - X_OFFSET) >4) {tagAlignState = TagAlignState.X_POS;}
                        if (Math.abs(tag.ftcPose.yaw - ROT_OFFSET) > 3) {tagAlignState = TagAlignState.ROTATE;}

                        break;
                }
            } else {
                tagAlignState = TagAlignState.NO_TAG;
            }

        } else {
            tagAlignState = TagAlignState.NO_TAG;
        }




        /*
         * Now we call our "Launch" function.
         */
        launch(gamepad2.rightBumperWasPressed());

        /*
         * Show the state and motor powers
         */
        telemetry.addData("State", launchState);
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
        telemetry.addData("motorSpeed", launcher.getVelocity());
        telemetry.addData("Launch State", launchState);
        telemetry.addData("Tag Team Blue?", isOnBlueTeam);

        telemetry.update();
    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
        vision.close();
    }


    //params
    // x = distance from goal (m)
    // theta = ramp angle (radians)
    // y = change in height: goal height - bot height (m)
    double getLaunchValue(double x, double theta, double y ){

        // C = D * pi
        double wheelCir = 0.095 * Math.PI;

        //calculates velocity based of kinematics equation
        double velocity = Math.sqrt((9.8 * x * x) /
                (2 * Math.cos(theta) * Math.cos(theta) * (x * Math.tan(theta) - y) ));

        // divides by wheel circumference to get angular velocity
        // multiplies of 360 to convert to degrees
        // divides by 19.23 to account for change in gear ratio
        return ((velocity / wheelCir) * 360) / (19.23 * MagicNumber);
    }



    void omniwheelDrive(double speed){
        double Pad2RightStickY = -gamepad2.right_stick_y;
        double LeftStickY = gamepad1.left_stick_y;
        double LeftStickX = -gamepad1.left_stick_x;
        double RightStickX = -gamepad1.right_stick_x;


        frontright.setPower((-RightStickX / 1.5 + (LeftStickY - LeftStickX)) * speed);
        backright.setPower((-RightStickX / 1.5 + (LeftStickY + LeftStickX)) * speed);
        frontleft.setPower((RightStickX / 1.5 + (LeftStickY + LeftStickX)) * speed);
        backleft.setPower((RightStickX / 1.5 + (LeftStickY - LeftStickX)) * speed);


    }



    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                    launcherTimer.reset();
                }
                break;
            case SPIN_UP:
                //launcher.setVelocity(45);
                //launcher.setPower(1);
                //if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                //    launchState = LaunchState.LAUNCH;
                //}
                if (launcherTimer.seconds() > LAUNCHER_TIME_SECONDS) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);
                    //launcher.setVelocity(0);
                    // launcher.setPower(0);
                }
                break;

        }
    }
}