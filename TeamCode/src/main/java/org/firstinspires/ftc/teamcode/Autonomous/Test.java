package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Control.AutonomousControl;
import org.firstinspires.ftc.teamcode.Control.Goal;

@Autonomous(name="Test Drivetrain", group = "basic")
public class Test extends AutonomousControl {
    private ElapsedTime runtime = new ElapsedTime();
//hi
    @Override
    public void runOpMode() throws InterruptedException {

        setup(runtime, Goal.setupType.teleop, Goal.setupType.claw);
        telemetry.addLine("Start!");
        telemetry.update();

        if (opModeIsActive()){

            rob.driveTrainEncoderMovement(0.2, 40, 10, 0, Goal.movements.right);

        }


    }
}