package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SwerveModule {

    private DcMotor driveMotor;
    private DcMotor swerveMotor;
    private AnalogInput potentiometer;

    private double swervePower = 0.5;

    private boolean driveReverse = false;

    private final double MAX_POTENTIOMETER_VOLTAGE = 3.3;

    private final int TICKS_PER_REV = 1124;
    private final double DEGREE_TO_TICKS_FACTOR = TICKS_PER_REV / 360.0;

    private final double VOLTAGE_TO_DEGREE_FACTOR = 360.0 / MAX_POTENTIOMETER_VOLTAGE;

    private final double GEAR_RATIO = 1;

    public final double angleFromCenter;

    public final double distFromCenter;

    public SwerveModule(DcMotor driveMotor, DcMotor swerveMotor, AnalogInput potentiometer,
                        double angleFromCenter, double distFromCenter)
    {
        this.driveMotor = driveMotor;
        this.driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.swerveMotor = swerveMotor;
        this.swerveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        this.potentiometer = potentiometer;

        this.angleFromCenter = angleFromCenter;
        this.distFromCenter = distFromCenter;
    }

    public void rotateByDegree(double angle)
    {
        swerveMotor.setPower(swervePower);

        int currentTicks = swerveMotor.getCurrentPosition();
        int targetTicks = (int)(currentTicks + angle * DEGREE_TO_TICKS_FACTOR * GEAR_RATIO);
        swerveMotor.setTargetPosition(targetTicks);
    }

    public void rotateToDegree(double angle)
    {
        double angleDiff = (angle - getCurrentAngle()) % 360;
        if(angleDiff < -180)
        {
            angleDiff += 360;
        }
        else if(angleDiff > 180)
        {
            angleDiff -= 360;
        }
        //we're now constrained -180 to +180, now deal with getting us reversed if needed
        if(angleDiff < -90)
        {
            angleDiff += 180;
            driveReverse = !driveReverse;
            //todo check if this maintains power
            driveMotor.setDirection(driveReverse ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        }
        else if(angleDiff > 90)
        {
            angleDiff -= 180;
            driveReverse = !driveReverse;
            //todo check if this maintains power
            driveMotor.setDirection(driveReverse ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        }
        //now we only will rotate a minimum of 90 degrees in either direction,
        //reversing the motor if we need to do so.
        rotateByDegree(angleDiff);
    }

    public void setSwervePower(double power)
    {
        swervePower = power;
    }

    public double getCurrentAngle()
    {
        return potentiometer.getVoltage() * VOLTAGE_TO_DEGREE_FACTOR;
    }

    public void setDriveMode(DcMotor.RunMode mode)
    {
        driveMotor.setMode(mode);
    }

    public void setDrivePower(double power)
    {
        driveMotor.setPower(power);
    }

    public int getDrivePosition()
    {
        return driveMotor.getCurrentPosition();
    }

    public void setDriveTarget(int target)
    {
        driveMotor.setTargetPosition(target);
    }
}
