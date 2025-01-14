package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.Range;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeouts)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

// Register this as an autonomous OpMode for the robot controller app
@Autonomous(name = "Auto", group = "Robot")
public class MyStarterBot2025Autonomous extends LinearOpMode {

    static final double COUNTS_PER_MOTOR_REV = 1024;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 1.0;     // No External Gearing.
    static final double WHEEL_DIAMETER_INCHES = 2.95;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 1;
    static final double TURN_SPEED = 1;
    static final double ARM_SPEED = 1.1;
    // Declare OpMode members for motors and servos.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor rightFrontWheel = null; // Forward
    private DcMotor rightRearWheel = null; // Forward
    private DcMotor leftFrontWheel = null; // Backward
    private DcMotor leftRearWheel = null; // Backward
    private DcMotor towerArm = null;
    private DcMotor foreArm1 = null; // Declare REV Core Hex Motor here
    private DcMotor foreArm2 = null; // Declare REV Core Hex Motor here
    private Servo claw = null;

    @Override
    public void runOpMode() {

        // Initialize hardware variables with names matching the robot configuration.
        rightFrontWheel = hardwareMap.get(DcMotor.class, "frontRight"); // Forward
        rightRearWheel = hardwareMap.get(DcMotor.class, "backRight"); // Forward
        leftFrontWheel = hardwareMap.get(DcMotor.class, "frontLeft"); // Backward
        leftRearWheel = hardwareMap.get(DcMotor.class, "backLeft"); // Backward

        towerArm = hardwareMap.get(DcMotor.class, "towerArm");
        foreArm1 = hardwareMap.get(DcMotor.class, "foreArm1"); // Initialize the Core Hex Motor
        foreArm2 = hardwareMap.get(DcMotor.class, "foreArm2"); // Initialize the Core Hex Motor

        claw = hardwareMap.get(Servo.class, "claw");

        // Set motor directions.
        rightFrontWheel.setDirection(DcMotor.Direction.FORWARD);
        rightRearWheel.setDirection(DcMotor.Direction.FORWARD);
        leftFrontWheel.setDirection(DcMotor.Direction.REVERSE);
        leftRearWheel.setDirection(DcMotor.Direction.REVERSE);

        rightFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRearWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFrontWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftRearWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        towerArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        foreArm1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        foreArm2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightRearWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftRearWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        towerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        foreArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        foreArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Display status and update telemetry.
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Starting at", "%7d :%7d :%7d :%7d",
                leftFrontWheel.getCurrentPosition(),
                leftRearWheel.getCurrentPosition(),
                rightFrontWheel.getCurrentPosition(),
                rightRearWheel.getCurrentPosition());

        telemetry.update();

        waitForStart();

        // Step through each leg of the path,
        // Note: Reverse movement is obtained by setting a negative distance (not speed)
        

        // FIRST HOOK (COMPLETED)
        encoderDrive(DRIVE_SPEED, 6.3, 6.3, 1.0);  // Move forward
        encoderDrive(TURN_SPEED, -10.5, 10.5, 1.0);  // Turn left
        encoderDrive(DRIVE_SPEED, 10,  10, 1.0);  // Move forward
        encoderDrive(TURN_SPEED, 10.6, -10.6, 1.0);  // Turn right
        encoderTowerArm(ARM_SPEED, 30, 3.5); // Towerarm lift
        encoderForeArms(ARM_SPEED, 44, 2); // Extend ForeArm forward
        claw.setPosition(0.0);                 //Open Claw
       
       // SECOND HOOK (IN PROGRESS)
        encoderForeArms(ARM_SPEED, -7, 1.0); // retract ForeArm backward
        encoderTowerArm(ARM_SPEED, -20, 1.0); // Towerarm down
        encoderDrive(TURN_SPEED, 10.5, -10.5, 1.0);  // Turn right
        encoderDrive(DRIVE_SPEED, 22,  22, 3.0);  // Move forward
        encoderDrive(TURN_SPEED, -10.5, 10.5, 1.0);  // Turn left        
        encoderDrive(DRIVE_SPEED, 4.9,  4.9, 2.4);  // Move forward
        claw.setPosition(1.0);  //Close Claw
        sleep(1000);
        encoderDrive(TURN_SPEED, 20, -20, 4.0);  // Turn right
        encoderDrive(DRIVE_SPEED, 10, 14, 4.0);  // Move forward
        sleep(1000);
        claw.setPosition(0.0);                 //Open Claw
        










        // encoderDrive(TURN_SPEED, 10.5, -10.5, 2.3);  // Turn right
        // encoderDrive(DRIVE_SPEED, 5,  5, 1.0);  // Move forward
        // encoderDrive(TURN_SPEED, 10.5, -10.5, 2.3);  // Turn right
        //  encoderDrive(DRIVE_SPEED, 10,  10, 1.0);  // Move forward
        //   encoderTowerArm(ARM_SPEED, 10, 3.5); // Towerarm lift
        // encoderForeArms(ARM_SPEED, 10, 3.5); // Extend ForeArm forward
        // claw.setPosition(1.0);                 //Close Claw
        
        // encoderDrive(DRIVE_SPEED, 3, 3, 2.0);  // Move forward
        // encoderDrive(TURN_SPEED, -10.5, 10.5, 2.0);  // Turn left
        // encoderDrive(DRIVE_SPEED, 10,  10, 1.0);  // Move forward
        // encoderDrive(TURN_SPEED, 10.5, -10.5, 2.3);  // Turn right
        // encoderDrive(DRIVE_SPEED, 4,  4, 1.0);  // Move forward
        // encoderTowerArm(ARM_SPEED, 20, 3.5); // Towerarm lift
        // encoderForeArms(ARM_SPEED, 40, 3.5); // Extend ForeArm forward
        //   encoderDrive(DRIVE_SPEED, 3,  3, 1.0);  // Move forward
        // claw.setPosition(0.0);                 //Open Claw
        // encoderForeArms(ARM_SPEED, -40, 3.5); // retract ForeArm backward
        // encoderTowerArm(ARM_SPEED, -23, 3.5); // Towerarm down
        //   encoderDrive(DRIVE_SPEED, -2, -2, 1.0);  // Move back
         
      
         
         
         
         //STRAFING --->
       // encoderYaw(DRIVE_SPEED, -7, 7, 5.0); // Strafe left
        //encoderDrive(DRIVE_SPEED, 5, 5, 5.0);  // Move forward
        //  encoderYaw(DRIVE_SPEED, -7, 7, 3.0);  // Strafe left
        //   claw.setPosition(0.0);                 //Open Claw
        //  encoderForeArms(ARM_SPEED, 1, 2.0); // Extend ForeArm forward
        //  encoderDrive(DRIVE_SPEED, 8,  8, 1.0);  // Move forward
        
        
        //WITHOUT STRAFING
        //  encoderDrive(DRIVE_SPEED, 1,  1, 1.0);  // Move forward
        //  encoderDrive(TURN_SPEED, -10.5, 10.5, 2.0);  // Turn left
        //  encoderDrive(DRIVE_SPEED, 7.5,  7.5, 1.0);  // Move forward
        //  encoderDrive(TURN_SPEED, 10.5, -10.5, 2.3);  // Turn right
        //  encoderForeArms(ARM_SPEED, 1, 2.0); // Extend ForeArm forward
        //  encoderDrive(DRIVE_SPEED, 4,  4, 2.0);  // Move forward
        //  claw.setPosition(1.0);                 //Close Claw
        //  encoderForeArms(ARM_SPEED, -5, 3.5); // retract ForeArm back
        //  encoderTowerArm(ARM_SPEED, 35, 3.5); // Towerarm lift
        //  encoderDrive(DRIVE_SPEED, -5,  -5, 1.0);  // Move backward
        //  encoderDrive(TURN_SPEED,  -13.5, 13.5, 2.0);  // Turn left
        //  encoderDrive(DRIVE_SPEED, 2,  2, 1.0);  // Move forward
        //  encoderForeArms(ARM_SPEED, 35, 3.5); // Extend ForeArm forward
        //  claw.setPosition(0.0);                 //Open Claw
        //  encoderForeArms(ARM_SPEED, -35, 3.5); // retract ForeArm back
        //  encoderTowerArm(ARM_SPEED, -35, 3.5); // Towerarm down
        //  encoderDrive(TURN_SPEED, 13.3, -13.3, 2.0);  // Turn right
        //  encoderForeArms(ARM_SPEED, 1, 1.0); // Extend ForeArm forward
        //  encoderDrive(DRIVE_SPEED, 5,  5, 1.0);  // Move forward
        //  claw.setPosition(1.0);                 //Close Claw
        //  encoderForeArms(ARM_SPEED, -5, 3.5); // retract ForeArm back
        //  encoderTowerArm(ARM_SPEED, 35, 3.5); // Towerarm lift
        //  encoderDrive(DRIVE_SPEED, -5,  -5, 1.0);  // Move backward
        //  encoderDrive(TURN_SPEED, -13.3, 13.3, 2.0);  // Turn left
        //  encoderForeArms(ARM_SPEED, 35, 3.5); // Extend ForeArm forward
        //  claw.setPosition(0.0);                 //Open Claw
        //  encoderForeArms(ARM_SPEED, -35, 3.5); // retract ForeArm back
        //  encoderTowerArm(ARM_SPEED, -35, 3.5); // Towerarm down
        //  encoderDrive(TURN_SPEED, 11.4, -11.4, 2.0);  // Turn right
        //  encoderForeArms(ARM_SPEED, 1, 1.0); // Extend ForeArm forward
        //  encoderDrive(DRIVE_SPEED, 6,  6, 1.0);  // Move forward
        //  claw.setPosition(1.0);                 //Close Claw
        //  encoderForeArms(ARM_SPEED, -5, 2.0); // retract ForeArm back
        //  encoderTowerArm(ARM_SPEED, 35, 3.5); // Towerarm lift
        //  encoderDrive(DRIVE_SPEED, -4,  -4, 1.0);  // Move backward
        //  encoderDrive(TURN_SPEED, -11.4, 11.4, 2.0);  // Turn left
        //  encoderForeArms(ARM_SPEED, 35, 3.5); // Extend ForeArm forward
        //  claw.setPosition(0.0);                 //Open Claw
        //  encoderForeArms(ARM_SPEED, -35, 3.5); // retract ForeArm back
        //  encoderTowerArm(ARM_SPEED, -35, 3.5); // Towerarm down
        


        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(60000);  //1 min  pause to display final telemetry message.

    }

    public void encoderTowerArm(double speed,
                                double inches,
                                double timeouts) {
        int newTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = towerArm.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            towerArm.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            towerArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            towerArm.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeouts) && (towerArm.isBusy())){
                // Display it for the driver.
                telemetry.addData("Running to", " %7d", newTarget);
                telemetry.addData("Currently at", " at %7d", towerArm.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            towerArm.setPower(0);

            // Turn off RUN_TO_POSITION
            towerArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }

    public void encoderForeArms(double speed,
                                double inches,
                                double timeouts) {
        int newTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newTarget = foreArm1.getCurrentPosition() + (int) (inches * COUNTS_PER_INCH);

            foreArm1.setTargetPosition(newTarget);
            foreArm2.setTargetPosition(newTarget);

            // Turn On RUN_TO_POSITION
            foreArm1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            foreArm2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            foreArm1.setPower(Math.abs(speed));
            foreArm2.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeouts) && (foreArm1.isBusy() || foreArm2.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Running to", " %7d", newTarget);
                telemetry.addData("Currently at", " at %7d :%7d", foreArm1.getCurrentPosition(), foreArm2.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            foreArm1.setPower(0);
            foreArm2.setPower(0);

            // Turn off RUN_TO_POSITION
            foreArm1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            foreArm2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }


    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeouts) {
        int newLeftTarget;
        int newRightTarget;

        // Ensure that the OpMode is still active
        if (opModeIsActive()) {

            // Determine new target position, and pass to motor controller
            newLeftTarget = leftFrontWheel.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newRightTarget = rightFrontWheel.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            leftFrontWheel.setTargetPosition(newLeftTarget);
            leftRearWheel.setTargetPosition(-newLeftTarget);
            rightFrontWheel.setTargetPosition(newRightTarget);
            rightRearWheel.setTargetPosition(-newRightTarget);

            // Turn On RUN_TO_POSITION
            leftFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            leftRearWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightFrontWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightRearWheel.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            // reset the timeout time and start motion.
            runtime.reset();
            leftFrontWheel.setPower(Math.abs(speed));
            leftRearWheel.setPower(Math.abs(speed));
            rightFrontWheel.setPower(Math.abs(speed));
            rightRearWheel.setPower(Math.abs(speed));

            while (opModeIsActive() && (runtime.seconds() < timeouts) && (leftFrontWheel.isBusy() || rightFrontWheel.isBusy())) {
                // Display it for the driver.
                telemetry.addData("Running to", " %7d :%7d", newLeftTarget, newRightTarget);
                telemetry.addData("Currently at", " at %7d :%7d",
                        leftFrontWheel.getCurrentPosition(), rightFrontWheel.getCurrentPosition());
                telemetry.update();
            }

            // Stop all motion;
            leftFrontWheel.setPower(0);
            leftRearWheel.setPower(0);
            rightFrontWheel.setPower(0);
            rightRearWheel.setPower(0);

            // Turn off RUN_TO_POSITION
            leftFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            leftRearWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightFrontWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            rightRearWheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        }
    }
    
    
}
