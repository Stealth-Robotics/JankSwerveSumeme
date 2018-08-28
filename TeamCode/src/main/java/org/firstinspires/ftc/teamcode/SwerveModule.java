package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

public class SwerveModule {

    private DcMotor driveMotor;
    private DcMotor swerveMotor;

    private double swervePower = 0.5;

    private boolean driveReverse = false;

    private int zeroDegreePosition;

    final double MAX_POTENTIOMETER_VOLTAGE = 3.3;

    final int TICKS_PER_REV = 1124;
    final int TICKS_TO_DEGREE_FACTOR = 360 / TICKS_PER_REV;
    final int DEGREE_TO_TICKS_FACTOR = TICKS_PER_REV / 360;

    final double VOLTAGE_TO_TICKS_FACTOR = TICKS_PER_REV / MAX_POTENTIOMETER_VOLTAGE;

    final double GEAR_RATIO = 1;

    private SwerveDrive swerveDrive;

    public SwerveModule(DcMotor driveMotor, DcMotor swerveMotor, AnalogInput potentiometer, SwerveDrive swerveDrive)
    {
        this.driveMotor = driveMotor;
        this.driveMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        this.swerveMotor = swerveMotor;
        this.swerveMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        zeroDegreePosition = (int)(-potentiometer.getVoltage() * VOLTAGE_TO_TICKS_FACTOR / GEAR_RATIO);

        this.swerveDrive = swerveDrive;
    }

    public void rotateByDegree(int angle)
    {
        swerveMotor.setPower(swervePower);

        int currentTicks = swerveMotor.getCurrentPosition();
        int targetTicks = (int)(currentTicks + angle * DEGREE_TO_TICKS_FACTOR * GEAR_RATIO);
        swerveMotor.setTargetPosition(targetTicks);
    }

    public void rotateToDegree(int angle)
    {
        int angleDiff = (angle - getCurrentAngle()) % 360;
        if (angleDiff > 270)
        {
            angleDiff -= 360;
            driveReverse = false;
        }
        else if (angleDiff > 180)
        {
            angleDiff -= 180;
            driveReverse = true;
        }
        else if (angleDiff < - 270)
        {
            angleDiff += 360;
            driveReverse = false;
        }
        else if (angleDiff < -180)
        {
            angleDiff += 180;
            driveReverse = true;
        }

        rotateByDegree(angleDiff);
    }

    public void setSwervePower(double power)
    {
        swervePower = power;
    }

    public int getCurrentAngle()
    {
        return (int)((swerveMotor.getCurrentPosition() - zeroDegreePosition) * TICKS_TO_DEGREE_FACTOR / GEAR_RATIO + swerveDrive.getHeading());
    }

    public void setDriveMode(DcMotor.RunMode mode)
    {
        driveMotor.setMode(mode);
    }

    public void setDrivePower(double power)
    {
        driveMotor.setPower((driveReverse) ? -power : power);
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
