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

import static java.lang.System.currentTimeMillis;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
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

@TeleOp(name="Parent Opmode", group="Linear Opmode")
@Disabled
public class MainParentOpMode extends LinearOpMode {

    // Declare OpMode members, hardware variables
    public ElapsedTime runtime = new ElapsedTime();

    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;

    private DcMotorSimple liftMotor = null;

    private Servo gripperServo = null;

    private DigitalChannel LimitSwitch = null;


    // gyro stuff
   // private ModernRoboticsI2cRangeSensor gyroSensor = null;

    BNO055IMU imu;
    Orientation angles = new Orientation();

    // private CRServo intakeServo = null;
    // private Servo shooterFlipper = null;



    // Global Variables/Constants
    double liftPower = 1;
    double liftStop = 0;
    double servoGripPosition = 0.4;

    // Lift Positions
    int pos0 = 0; // bottom
    int pos1= 479;    //Just Above ConÉ
    int pos2= 3318;
    int pos3= 5687;
    int pos4= 7804;  //Top
    int posmax= 7850; // MAX
    int LiftOffSet= 0;  // Initialize to 0

    boolean goToPos1 = false;
    boolean goTopos2 = false;
    boolean goToPos3 = false;
    boolean goTopos4 = false;

    public void initialize(){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Driver Station app or Driver Hub).
        rightFront = hardwareMap.get(DcMotor.class, "rf_drive");
        leftFront = hardwareMap.get(DcMotor.class, "lf_drive");
        leftBack = hardwareMap.get(DcMotor.class, "lb_drive");
        rightBack = hardwareMap.get(DcMotor.class, "rb_drive");

        liftMotor = hardwareMap.get(DcMotorSimple.class, "lift_motor");

        gripperServo = hardwareMap.get(Servo.class, "gripper_servo");

        LimitSwitch = hardwareMap.get(DigitalChannel.class, "lift_limit_switch");

        gyroInitialize();

        //Set motor run mode (if using SPARK Mini motor controllers)


        //Set Motor  and servo Directions
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);


        liftMotor.setDirection(DcMotor.Direction.REVERSE);
        //liftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);



        //gripperServo.setDirection(Servo.Direction.FORWARD);


        //Set brake or coast modes.
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //BRAKE or FLOAT (Coast)
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //BRAKE or FLOAT (Coast)
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //BRAKE or FLOAT (Coast)
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //BRAKE or FLOAT (Coast)


        //Homing();
        //quick, dirty test
        /*
        for(int i =0; i < 200;i ++){
            Homing();
        }

 */
        //Update Driver Station Status Message after init
        telemetry.addData("Status:", "Initialized");
        telemetry.update();
       // telemetry.addData("current lift position", liftMotor.getCurrentPosition());
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
    private boolean GripCloseButton(){return gamepad1.left_bumper|| gamepad2.left_bumper;}
    private float right_trigger(){return gamepad1.right_trigger;}
    private boolean GripOpenButton(){
        if(gamepad1.left_trigger>.1||gamepad2.left_trigger>.1){
            return true;}
        else{
            return  false;}
    }
    private boolean b_button(){return gamepad1.b;}
    private boolean y_button(){return gamepad1.y;}
    private boolean a_button(){return gamepad1.a;}
    private boolean x_button(){return gamepad1.x;}


    private boolean liftDown_button(){return gamepad2.right_bumper;}
    private boolean liftUp_button(){
        if(gamepad2.right_trigger>.5){
            return true;
        }
        else{
            return false;
        }
    }

    private boolean pos1_button(){return gamepad2.a;}
    private boolean pos2_button(){return gamepad2.x;}
    private boolean pos3_button(){return gamepad2.b;}
    private boolean pos4_button(){return gamepad2.y;}

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
   Orientation Angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
   double heading = Angles.firstAngle;
   return heading;
    }

    public String formatDegrees(double degrees){
        return String.format(Locale.getDefault(), "%.1f",AngleUnit.DEGREES.normalize(degrees));
    }


    public void HolonomicDrive(){
        double motorspeedRF;
        double motorspeedRB;
        double motorspeedLF;
        double motorspeedLB;

        double PIOFFSET = Math.PI/4; //45 degrees

        double robotAngle= Math.atan2(gamepad1.left_stick_y,-gamepad1.left_stick_x)-Math.PI/2;

        double robotSpeed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

        double rotation = -gamepad1.right_stick_x;

        motorspeedLF = (robotSpeed*Math.cos(robotAngle+PIOFFSET)) + rotation;
        motorspeedRF = (robotSpeed*Math.sin(robotAngle+PIOFFSET)) - rotation;
        motorspeedLB = (robotSpeed*Math.sin(robotAngle+PIOFFSET)) + rotation;
        motorspeedRB = (robotSpeed*Math.cos(robotAngle+PIOFFSET)) - rotation;

        leftFront.setPower(motorspeedLF);
        leftBack.setPower(motorspeedLB);
        rightBack.setPower(motorspeedRB);
        rightFront.setPower(motorspeedRF);
/*
        if (resetGyroButton()){
            gyroInitialize();
        }
        */


    }

    public void FieldCentricDrive(){
        double motorspeedRF;
        double motorspeedRB;
        double motorspeedLF;
        double motorspeedLB;

        double PIOFFSET = Math.PI/4; //45 degrees

        double robotAngle= Math.atan2(gamepad1.left_stick_y,-gamepad1.left_stick_x)-(Math.PI/2)-(Math.toRadians(getAngle()));

        double robotSpeed = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);

        double rotation = -gamepad1.right_stick_x;


        motorspeedLF = (robotSpeed*Math.cos(robotAngle+PIOFFSET)) + rotation;
        motorspeedRF = (robotSpeed*Math.sin(robotAngle+PIOFFSET)) - rotation;
        motorspeedLB = (robotSpeed*Math.sin(robotAngle+PIOFFSET)) + rotation;
        motorspeedRB = (robotSpeed*Math.cos(robotAngle+PIOFFSET)) - rotation;

        leftFront.setPower(motorspeedLF);
        leftBack.setPower(motorspeedLB);
        rightBack.setPower(motorspeedRB);
        rightFront.setPower(motorspeedRF);

        if (resetGyroButton()){
            gyroInitialize();
        }



    }


    public void gyroInitialize(){

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        // TEST LATER YOU parameters.mode = BNO055IMU.SensorMode.GYRONLY;

        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);


        while(!isStopRequested() && !imu.isGyroCalibrated()){
            sleep(500);
            idle();
        }
    }






    /*****************************/
    //More Methods (Functions)

    //LIFT METHODS

    public void stopLift(){
        liftMotor.setPower(liftStop);
    }

    public boolean LiftAtBottom(){
        return LimitSwitch.getState();
    }

    public boolean liftAtTop(){
        if (GetLiftPosition() >posmax){
            return  true;
        }
        else{
            return false;
        }
    }

    /*
    public void ManuelLiftMove() {
        if (liftDown_button() == true) {
            liftMotor.setPower(-liftPower);
            telemetry.addData("going down :0", "");
        } else if (liftUp_button()){
            liftMotor.setPower(liftPower);
            telemetry.addData("going up :0", "");
        }
        else{
            stopLift();
        }
    }
*/

    public void liftUp(){
        if (liftUp_button()){
            liftMotor.setPower(liftPower);
            telemetry.addData("going up :0", "");
        }/*
        else {
            stopLift();
        }*/

    }

public int GetLiftRealPos(){
    return -rightBack.getCurrentPosition();
}

public int GetLiftPosition() {
    return GetLiftRealPos() - LiftOffSet;
}

public void SetLiftOffSet(){
        LiftOffSet = GetLiftRealPos();

}

public void Homing (){
        while((!LiftAtBottom()&&opModeIsActive())||(!LiftAtBottom()&&opModeInInit())){
            liftMotor.setPower(-0.3);
        }
        SetLiftOffSet();
}

public void GoToPositionUp(int tx){
    if (GetLiftPosition()  <= tx){
        liftMotor.setPower(liftPower);
        telemetry.addData("Up To ",tx);
        telemetry.update();
    }
}

public void GoToPositionDown(int targetsGotDeals){
    if (GetLiftPosition() >= targetsGotDeals){
        liftMotor.setPower(-liftPower*.5);
        telemetry.addData("Down To ",targetsGotDeals);
        telemetry.update();
    }

}

public void GotoPosition(int posL){
       if (GetLiftPosition() > posL+200){
           liftMotor.setPower(-liftPower*.7);
       }
       else {
           if (GetLiftPosition() < posL-200){
               liftMotor.setPower(liftPower);
           }
           else {
               liftMotor.setPower(0);
               goToPos1 =false;
               goTopos2 =false;
               goToPos3 =false;
               goTopos4 =false;
           }
       }
}

public void GoToPosManual(){
        if (pos1_button()){
            goToPos1 =true;
            goTopos2 =false;
            goToPos3 =false;
            goTopos4 =false;
        }
        if (pos2_button()){
            goToPos1 =false;
            goTopos2 =true;
            goToPos3 =false;
            goTopos4 =false;

        }
        if (pos3_button()){
            goToPos1 =false;
            goTopos2 =false;
            goToPos3 =true;
            goTopos4 =false;

        }
        if (pos4_button()){
            goToPos1 =false;
            goTopos2 =false;
            goToPos3 =false;
            goTopos4 =true;

        }

        if (liftUp_button() && !liftAtTop()){
            liftMotor.setPower(liftPower);
            goToPos1 =false;
            goTopos2 =false;
            goToPos3 =false;
            goTopos4 =false;
        } else{
            if (liftDown_button() && !LiftAtBottom()){
                liftMotor.setPower(-liftPower*.7);
                goToPos1 =false;
                goTopos2 =false;
                goToPos3 =false;
                goTopos4 =false;
            }
            else if(goToPos1) {
                GotoPosition(pos1);
            }
            else if(goTopos2){
                GotoPosition(pos2);
            }
            else if(goToPos3){
                GotoPosition(pos3);
            }
            else if(goTopos4){
                GotoPosition(pos4);
            }
            else {
                liftMotor.setPower(0);
            }
        }

}
 //hlol



    /*public void goToPos(){
        if (pos1_button() == true) {
            GoToPositionDown(pos1);
        }
        else {
            if (pos2_button() == true){
                if (GetLiftPosition() <= pos2-10) {
                    GoToPositionUp(pos2);
                }

                if (GetLiftPosition() >= pos2+10){
                    GoToPositionDown(pos2);
                }
                else {
                    if (pos3_button() == true){
                        if (GetLiftPosition() <=pos3-10) {
                            GoToPositionUp(pos3);
                        }
                        if (GetLiftPosition() >=pos3+10){
                            GoToPositionDown(pos3);
                        }
                        else {
                            if (pos4_button() == true){
                                GoToPositionUp(pos4);
                            }
                            else {
                                stopLift();
                            }
                        }
                    }
                }
            }
        }
    }
*/

    //Look into doing a different way with variable "flags"
    // There does not seem to be a good way to avoid jumpy behavior and allow it to stop where it should
    // as well as allowing manual up/down controls
    // --Matt
    public void LiftMoveButtons(){
        if (pos1_button() == true) {
            if(GetLiftPosition() > pos1+40){
                GoToPositionDown(pos1);
            }
            else if (GetLiftPosition() < pos1-40){
                GoToPositionUp(pos1);
            }
        }

        else {
            if (pos2_button() == true){
                if (GetLiftPosition() <= pos2-10) {
                    GoToPositionUp(pos2);
                }

                if (GetLiftPosition() >= pos2+10){
                    GoToPositionDown(pos2);
                }
                else {
                    if (pos3_button() == true){
                        if (GetLiftPosition() <=pos3-10) {
                            GoToPositionUp(pos3);
                        }
                        if (GetLiftPosition() >=pos3+10){
                            GoToPositionDown(pos3);
                        }
                        else {
                            if (pos4_button() == true){
                                GoToPositionUp(pos4);
                            }
                            else {
                                stopLift();
                                /*
                                if (liftDown_button() == true) {
                                    liftMotor.setPower(-liftPower);
                                    telemetry.addData("going down :0", "");
                                } else if (liftUp_button() >= 0.5){
                                    liftMotor.setPower(liftPower);
                                    telemetry.addData("going up :0", "");
                                }
                                else{
                                    stopLift();
                                }
                                */
                            }
                        }
                    }
                }
            }
        }
    }


    public void goToPos2(){
        if (pos2_button() == true){
            if (GetLiftPosition() <= pos2-10) {
                GoToPositionUp(pos2);
            }

            if (GetLiftPosition() >= pos2+10){
                GoToPositionDown(pos2);
            }
            else {
                stopLift();
            }
        }
    }

    public void goToPos3(){
        if (pos3_button() == true){
            if (GetLiftPosition() <=pos3-10) {
                GoToPositionUp(pos3);
            }
            if (GetLiftPosition() >=pos3+10){
                GoToPositionDown(pos3);
            }
            else {
                stopLift();
            }
        }
    }

    public void goToPos4(){
        if (pos4_button() == true){
            GoToPositionUp(pos4);
        }
        else {
            stopLift();
        }
    }




//Gripper Methods

public void GripperOpen() {
    if (GripOpenButton() == true) {
        gripperServo.setPosition(0);

    }
}
public void GripperClose(){
    if (GripCloseButton() == true) {
        gripperServo.setPosition(servoGripPosition);
    }
}

public void GripperFunction() {
        GripperClose();
        GripperOpen();
}

    /*****************************/
    //Autonomous Functions




    public void AUTO_Drive(double robotAngle, double robotSpeed, double rotation, long millisecondTime) {
        double motorspeedRF;
        double motorspeedRB;
        double motorspeedLF;
        double motorspeedLB;

        robotAngle = Math.toRadians(robotAngle);

        double PIOFFSET = Math.PI / 4;

        motorspeedLF = (robotSpeed*Math.cos(robotAngle+PIOFFSET)) + rotation;
        motorspeedRF = (robotSpeed*Math.sin(robotAngle+PIOFFSET)) - rotation;
        motorspeedLB = (robotSpeed*Math.sin(robotAngle+PIOFFSET)) + rotation;
        motorspeedRB = (robotSpeed*Math.cos(robotAngle+PIOFFSET)) - rotation;



        leftFront.setPower(motorspeedLF);
        leftBack.setPower(motorspeedLB);
        rightBack.setPower(motorspeedRB);
        rightFront.setPower(motorspeedRF);

        sleep(millisecondTime);
        stopDrive();


    }



public void AUTO_gripperClose(){
        gripperServo.setPosition(0);
}
public void AUTO_gripperOpen(){
        gripperServo.setPosition(servoGripPosition);
}

public void AUTO_GoToPosition1(){
        GoToPositionDown(pos1);
}

    public void AUTO_GoToPosition2(){
        GoToPositionUp(pos2);
    }

    public void AUTO_GoToPosition3(){
        GoToPositionUp(pos3);
    }

    public void AUTO_GoToPosition4(){
        GoToPositionUp(pos4);
    }






                /*****************************/
    //Encoder Functions
   /*
    public double getLeftVerticalEncoder(){
        return rightFront.getCurrentPosition();
    }
    */

    /*****************************/


}
