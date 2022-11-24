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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.util.ElapsedTime;

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

@TeleOp(name="Parent Opmode Test", group="Linear Opmode")
@Disabled
public class ParentOpModeTesting extends LinearOpMode {

    // Declare OpMode members, hardware variables
    public ElapsedTime runtime = new ElapsedTime();

    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;

    public DcMotorSimple liftMotor = null;

    private DigitalChannel liftLimitSw = null;

    //private Lift liftSubsystem = new Lift(lift);

    //Other Global Variables
    //put global variables here...
    //
    int liftMax = 6800;
    int liftMin = 100;
    int lift_encoder_offset = 0;
    int lift_destination = 0; // May or may not use

    int liftpos0 = 1000;
    int liftpos1 = 3000;
    int liftpos2 = 5000;
    int liftpos3 = 6500;



    public void initialize(){
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Driver Station app or Driver Hub).
        rightFront = hardwareMap.get(DcMotor.class, "rf_drive");
        leftFront = hardwareMap.get(DcMotor.class, "lf_drive");
        leftBack = hardwareMap.get(DcMotor.class, "lb_drive");
        rightBack = hardwareMap.get(DcMotor.class, "rb_drive");

        liftMotor = hardwareMap.get(DcMotorSimple.class, "lift_motor");
        liftLimitSw = hardwareMap.get(DigitalChannel.class,"lift_limit_switch");

        //Set Directions
        liftMotor.setDirection(DcMotor.Direction.REVERSE);

        //Set brake or coast modes.
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //BRAKE or FLOAT (Coast)

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

        // Init loop - optional
        while(opModeInInit()){
            // Code in here will loop continuously until OpMode is started
        }

        // Wait for the game to start (driver presses PLAY) - May not be needed if using an init Loop
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // code here should never actually execute in parent opmode.
            // This function will be be overridden by child opmode classes


            //include emergency stop check in all runOpMode() functions/methods
            //implementation depends on which E-stop function will be used (boolean/void)
            if(emergencyStopped()){
                //terminateOpModeNow();   // New method in 2022. (Immediately, Cleanly exits OpMode)
                break;
            }

            //checkEmergencyStop(); // Stops motors and Terminates if buttons are pressed
            //without additional code in the while(opModeIsActive) loop.

            telemetry.update();
        }
    }

    /*****************************/
    //Controls should be mapped here to avoid accessing gamepad directly in other functions/methods
    //This also makes it simpler to re-map controls as desired
    //CONTROLLER MAP

    // Thumbsticks
    public double left_sticky_x(){
        return gamepad1.left_stick_x;
    }

    public double left_sticky_y(){
        return -gamepad1.left_stick_y;
    }


    // Buttons
    public boolean emergencyButtons(){
        // check for combination of buttons to be pressed before returning true
        return true;
    }

    public boolean pushButton(){
        return gamepad1.x;
    }
    public boolean pos0Button(){
        return gamepad1.a;
    }
    public boolean pos1Button(){
        return gamepad1.x;
    }
    public boolean pos2Button(){
        return gamepad1.b;
    }
    public boolean pos3Button(){
        return gamepad1.y;
    }

    public boolean liftDownButton(){return gamepad1.left_bumper;}
    public boolean liftUpButton(){return gamepad1.right_bumper;}

    public double liftTriggers(){
        if(gamepad1.left_trigger > 0.2){
            return -gamepad1.left_trigger;
        }
        else{
            if(gamepad1.right_trigger > 0.2){
                return gamepad1.right_trigger;
            }
            else{
                return 0;
            }
        }
    }

    public boolean triggerButton(){
        if((gamepad1.right_trigger>.25)||(gamepad2.right_trigger>.25)){
            return true;         // Converts analog triggers into digital button presses (booleans)
        }
        else{
            return false;
        }
    }


    /****************************/
    // Emergency Stop Functions
    // Only one is needed.
    // If using boolean version, call to function will need to be
    // placed in conditional (if/then) statement with code to break from loop or terminate opmode.

    public boolean emergencyStopped(){
        if (emergencyButtons()) {
            //stop all motors, servos, etc.
            return true;
        }
        else {
            return false;
        }
    }

    public void checkEmergencyStop(){
        if(emergencyButtons()){
            //stop all motors, servos, etc.
            terminateOpModeNow();   // Force exit of OpMode
        }
    }



    /*****************************/
    //Drive Methods

    // Assign left and right drive speed using arguments/parameters rather than hardcoding
    // thumb stick values inside function body. This will allow tank drive to be reused for
    // autonomous programs without additional work
    public void tankdrive(double left, double right){

    }




    /*****************************/
    //More Methods (Functions)



    /*****************************/
    //Autonomous Functions

    /*****************************/
    //Gyro Functions


    /*****************************/
    //Encoder Functions
   /*
    public double getLeftVerticalEncoder(){
        return rightFront.getCurrentPosition();
    }
    */

    public int getLiftPosition(){
        int liftPosition = rightBack.getCurrentPosition() - lift_encoder_offset;
        return liftPosition;
    }

    public int getLiftPositionReal(){
        //get lift real lift position without factoring in offset
        return rightBack.getCurrentPosition();
    }

    public void setLiftEncoderOffset(){
        lift_encoder_offset = getLiftPositionReal();
    }

    /*****************************/
    //Lift Functions

    public void stopLift(){
        liftMotor.setPower(0);
    }

    public void liftHomingRoutine(){
        double homingSpeed = 0.3;

        while(!liftAtBottom() && opModeIsActive()){
            liftMotor.setPower(homingSpeed);
        }
    }


    public boolean liftAtBottom(){
        boolean limitSwitch = liftLimitSw.getState();

        telemetry.addData("Lift Limit Switch: ",limitSwitch);

        return limitSwitch;
    }



    public void liftManualAnalogControl(double liftSpeed){
        double upSpeedScaler = 1;
        double downSpeedScaler = 0.5;

        telemetry.addData("Lift Speed: ",liftSpeed);

        if(liftSpeed > 0){
            if(getLiftPosition()<liftMax){
                double liftUpSpeed = liftSpeed*upSpeedScaler;
                liftMotor.setPower(liftUpSpeed);
            }
        }
        else{
            if(getLiftPosition()>liftMin){
                double liftDownSpeed = -liftSpeed*downSpeedScaler;
                liftMotor.setPower(liftDownSpeed);
            }
        }
    }

    public void liftManualNoSpeedControl(){
        double LIFT_SPEED = 1;
        // Lift Speed limited by scaling factors in lift AnalogControl function

        if(liftUpButton()){
            liftManualAnalogControl(LIFT_SPEED);
        }
        else{
            if(liftDownButton()){
                liftManualAnalogControl((-LIFT_SPEED));
            }
            else{
                stopLift();
            }
        }

    }

    public void goToButtonPositionWhileHeld(){
        if(pos0Button()){
            goToPosition(liftpos0);
        }
        else{
            if(pos1Button()){
                goToPosition(liftpos1);
            }
            else{
                if(pos2Button()){
                    goToPosition(liftpos2);
                }
                else{
                    if (pos3Button()) {
                        goToPosition(liftpos3);
                    }
                    else{
                        stopLift();
                    }
                }
            }
        }
    }


    public void set_destination_WithButtons(){
        if(pos0Button()){
            lift_destination = liftpos0;
        }
        if(pos1Button()){
            lift_destination = liftpos1;
        }
        if(pos2Button()){
            lift_destination = liftpos2;
        }
        if(pos3Button()){
            lift_destination = liftpos3;
        }

    }

    // Probably need to incorporate a loop here.
    // Does not send stop command unless destination has already been reached when function is called.
    // (IE: function needs to be called repeatedly until destination is reached otherwise, it will overshoot)
    // Could make destination a global variable in opmode, and call goToPosition(destination) every cycle.
    // This would mean buttons would change destination, rather than calling a different goToPosition.
    // Still, it would interrupt manual controls unless manual controls incremented destination position,
    // rather than setting motor power.
    public void goToPosition(int destination){
        int liftDeadZone_Upper = 300;
        int liftDeadZone_Lower = 200;
        double liftUpSpeed = 1;
        double liftDownSpeed = -0.5;

        // Optional, but not a bad idea:
        // Catch any attempts to pass limits
        if((destination > liftMax) || (destination < liftMin)){
            return;
        }

        // Move the lift
        if(getLiftPosition() < destination - liftDeadZone_Lower){
            //go up
            liftMotor.setPower(liftUpSpeed);
            telemetry.addData("Lift: Going to position: ",destination);
        }
        else{
            if(getLiftPosition() > destination + liftDeadZone_Upper){
                //go down
                liftMotor.setPower(liftDownSpeed);
                telemetry.addData("Lift: Going to position  ",destination);
            }
            else {
                stopLift();
                telemetry.addData("Lift: ", "Stopped");
            }
        }

    }



}