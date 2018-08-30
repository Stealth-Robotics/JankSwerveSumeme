package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class TriSwerveDrive implements Runnable {

    private SwerveModule[] modules;
    BNO055IMU imu;

    double speed;
    double xSpeed;
    double ySpeed;
    double rotSpeed;

    DcMotor.RunMode mode;

    public TriSwerveDrive(
            DcMotor drive1, DcMotor swerve1, AnalogInput potentiometer1,
            DcMotor drive2, DcMotor swerve2, AnalogInput potentiometer2,
            DcMotor drive3, DcMotor swerve3, AnalogInput potentiometer3,
            BNO055IMU imu
            )
    {
        modules = new SwerveModule[3];
        modules[0] = new SwerveModule(drive1, swerve1, potentiometer1, 0, 18);
        modules[1] = new SwerveModule(drive2, swerve2, potentiometer2, 120, 18);
        modules[2] = new SwerveModule(drive3, swerve3, potentiometer3, -120, 18);

        this.imu = imu;

        mode = DcMotor.RunMode.RUN_WITHOUT_ENCODER;
    }

    @Override
    public void run()
    {
        if (speed != 0)
        {
            double angle;
            if (xSpeed != 0)
            {
                angle = Math.atan(ySpeed / xSpeed);
            }
            else
            {
                angle = (ySpeed >= 0) ? 90 : -90;
            }

            for (SwerveModule module : modules)
            {
                module.rotateToDegree(angle);
                module.setDrivePower(speed);
            }
        }
        else if (rotSpeed != 0)
        {
            for (SwerveModule module : modules)
            {
                module.rotateToDegree(module.angleFromCenter - 90);
                module.setDrivePower(rotSpeed);
            }
        }
        else
        {

        }
    }

    public void setRot(double rotSpeed)
    {
        if (rotSpeed + this.speed > 1)
        {
            double speedFactor = 1 / (rotSpeed + this.speed);
            setSpeed(this.speed * speedFactor);
            rotSpeed *= speedFactor;
        }

        this.rotSpeed = rotSpeed;
    }

    public void setSpeed(double speed)
    {
        if (speed + this.rotSpeed > 1)
        {
            double speedFactor = 1 / (speed + this.rotSpeed);
            setRot(this.rotSpeed * speedFactor);
            speed *= speedFactor;
        }

        this.speed = speed;
    }

    public void setDirection(double angle)
    {
        xSpeed = Math.cos(angle) * speed;
        ySpeed = Math.sin(angle) * speed;
    }

    public void setVelocity(double angle, double speed)
    {
        setSpeed(speed);
        setDirection(angle);
    }

    public void setMode(DcMotor.RunMode mode)
    {
        for (SwerveModule module : modules)
        {
            module.setDriveMode(mode);
        }

        this.mode = mode;
    }

    public double[] findNextVector(SwerveModule module)
    {
        //definately not complete, do not pay attention to this
        //am going to finish it, but am tired and going to bed

        double xCoord = module.distFromCenter * Math.cos(rotSpeed / module.distFromCenter) + xSpeed;
        double yCoord = module.distFromCenter * Math.sin(rotSpeed / module.distFromCenter) + ySpeed;
        double[] vector = new double[2];
        vector[0] = Math.sqrt(xCoord * xCoord + yCoord * yCoord);
        vector[1] = yCoord / xCoord;
        return vector;
    }
}
