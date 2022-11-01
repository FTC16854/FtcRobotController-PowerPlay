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

import android.graphics.drawable.GradientDrawable;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;
import com.qualcomm.robotcore.hardware.OrientationSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Locale;

/**
 * Original FTC opmode header block
 *
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 **/

/** Parent OpMode Class
 * All Teleop and Autonomous OpModes should inherit from (extend) ParentOpMode.
 * Each child/subclass OpMode should have its own unique runOpMode() method that will
 * override the ParentOpMode runOpMode() method.
 **/

@TeleOp(name="Parent Opmode Example", group="Linear Opmode")
@Disabled
public class MainParentOpMode extends LinearOpMode {

    // Declare OpMode members, hardware variables
    public ElapsedTime runtime = new ElapsedTime();

    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;

    private DcMotor liftMotor = null;

    private Servo gripperServo = null;


    // gyro stuff
   // private ModernRoboticsI2cRangeSensor gyroSensor = null;

    BNO055IMU gyroSensorInterface;
    Orientation angles = new Orientation();

    // private CRServo intakeServo = null;
    // private Servo shooterFlipper = null;



    // Global Variables/Constants
    double liftPower = 0.7;
    double servoGripPosition = 0.7;






    public void initialize(){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Driver Station app or Driver Hub).
        rightFront = hardwareMap.get(DcMotor.class, "rf_drive");
        leftFront = hardwareMap.get(DcMotor.class, "lf_drive");
        leftBack = hardwareMap.get(DcMotor.class, "lb_drive");
        rightBack = hardwareMap.get(DcMotor.class, "rb_drive");

        liftMotor = hardwareMap.get(DcMotor.class, "lift_motor");

        gripperServo = hardwareMap.get(Servo.class, "gripper_servo");




        //Set motor run mode (if using SPARK Mini motor controllers)


        //Set Motor  and servo Directions
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);


        liftMotor.setDirection(DcMotor.Direction.FORWARD);

        //gripperServo.setDirection(Servo.Direction.FORWARD);

        //Set range for special Servos
        //wobbleLift.scaleRange(0.15,.85); //Savox PWM range is between 0.8 and 2.2 ms. REV Hub puts out 0.5-2.5ms.

        //Set brake or coast modes.
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //BRAKE or FLOAT (Coast)
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //BRAKE or FLOAT (Coast)
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //BRAKE or FLOAT (Coast)
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //BRAKE or FLOAT (Coast)

        //Update Driver Station Status Message after init
        telemetry.addData("Status:", "Initialized");
        telemetry.update();
    }


    /**
     * runOpMode() will be overridden in child OpMode.
     * Basic structure should remain intact (init, wait for start, while(opModeIsActive),
     * Additionally, Emergency conditions should be checked during every cycle
     */
    @Override
    public void runOpMode() {

        initialize();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // code here should never actually execute in parent opmode.
            // This function will be be overridden by child opmode classes


            //include emergency stop check in all runOpMode() functions/methods
                //implementation depends on which E-stop function will be used (boolean/void)

            //checkEmergencyStop(); // Stops motors and Terminates if buttons are pressed
            //without additional code in the while(opModeIsActive) loop.

            telemetry.update();
        }
    }

    /*****************************/
    //Controls should be mapped here to avoid
    //CONTROLLER MAP

    // Thumbsticks
    private double leftStick_X(){
        return gamepad1.left_stick_x;
        }
    private double leftStick_Y(){
        return gamepad1.left_stick_y;
    }

    private double rightStick_X() {
        return gamepad1.right_stick_x;
    }
    private double rightStick_Y() {
        return gamepad1.right_stick_y;
    }


    // Buttons
    public boolean emergencyButtons(){
        // check for combination of buttons to be pressed before returning true
        return (gamepad1.b&&gamepad1.y) || (gamepad2.b&&gamepad2.y);
    }

    // Rename these based on function
    private boolean right_bumper(){return gamepad1.right_bumper;}
    private boolean left_bumper(){return gamepad1.left_bumper;}
    private boolean b_button(){return gamepad1.b;}
    private boolean y_button(){return gamepad1.y;}
    private boolean a_button(){return gamepad1.a;}
    private boolean x_button(){return gamepad1.x;}

    public boolean resetGyroButton(){return (a_button()&& b_button());}



    /**
     * @return**************************/
    // Emergency Stop Functions
        // Only one is needed.
        // If using boolean version, call to function will need to be
        // placed in conditional (if/then) statement with code to break from loop or terminate opmode.

    public void checkEmergencyStop() {
        if (emergencyButtons()) {
            terminateOpModeNow();
        }
    }

    /*****************************/
    //Drive Methods

    // Assign left and right drive speed using arguments/parameters rather than hardcoding
    //    // thumb stick values inside function body. This will allow tank drive to be reused for
    //    // autonomous programs without additional work
    public void tankDriving(double left, double right){
        rightFront.setPower(right);
        rightBack.setPower(right);
        leftFront.setPower(left);
        leftBack.setPower(left);
    }

    public void stopDrive(){
        tankDriving(0,0);
    }

    public double getAngle() {
   Orientation Angles = gyroSensorInterface.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
   double heading = Angles.firstAngle;
   return heading;
    }

    public String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f",AngleUnit.DEGREES.normalize(degrees));
    }


    public void FeildcentricDrive(){
        double motorspeedRF;
        double motorspeedRB;
        double motorspeedLF;
        double motorspeedLB;

        double PIOFFSET = Math.PI/4;

        double robotAngle= Math.atan2(gamepad1.left_stick_y,gamepad1.left_stick_x)-PIOFFSET;

        double robotSpeed = Math.hypot(gamepad1.left_stick_x, -gamepad1.left_stick_y);

        double rotation = gamepad1.right_stick_x;

        motorspeedRF = (robotSpeed*Math.sin(robotAngle+PIOFFSET)) - rotation;
        motorspeedRB = (robotSpeed*Math.cos(robotAngle+PIOFFSET)) - rotation;
        motorspeedLF = (robotSpeed*Math.cos(robotAngle+PIOFFSET)) + rotation;
        motorspeedLB = (robotSpeed*Math.sin(robotAngle+PIOFFSET)) + rotation;

        leftFront.setPower(motorspeedLF);
        leftBack.setPower(motorspeedLB);
        rightBack.setPower(motorspeedRB);
        rightFront.setPower(motorspeedRF);

        if (resetGyroButton()){
            gyroInitialize();
        }

    }

    public void gyroInitialize(){
        gyroSensorInterface.getParameters();
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        // TEST LATER YOU parameters.mode = BNO055IMU.SensorMode.GYRONLY;

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        gyroSensorInterface = hardwareMap.get(BNO055IMU.class, "gyro_sensor_interface");
        gyroSensorInterface.initialize(parameters);


        while(!isStopRequested() && !gyroSensorInterface.isGyroCalibrated()){
            sleep(500);
            idle();
        }
    }

/*
TODO
    1. Auto Park In Terminal Function
     a. move to the right - make function
    2. Auto substation park
     a. move left - make function
    3. Auto Low Junction
       a. move left til junction - make function
       b. lift up - make
       c. move forward - make
       d. lift down - make
       e. drop cone - make
       f. lift up
       g. back up - make function
       h. lift down
       i. Park In Terminal
       1203618
 */



    public void Amongus(){

    }



    /*****************************/
    //More Methods (Functions)
//TODO add if statement for if button is pressed
    //LIFT METHODS
public void liftUp(){
    liftMotor.setPower(liftPower);
}

public void liftDown(){
    liftMotor.setPower(-liftPower);
}

public void GripperIn(){
    gripperServo.setPosition(servoGripPosition);
}

public void GripperOut(){
    gripperServo.setPosition(0);
}


    /*****************************/
    //Autonomous Functions

    /*****************************/
    //Encoder Functions
   /*
    public double getLeftVerticalEncoder(){
        return rightFront.getCurrentPosition();
    }
    */

    /*****************************/


}
