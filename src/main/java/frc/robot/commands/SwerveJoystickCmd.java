package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.Swerve.DriveSubsystem;

public class SwerveJoystickCmd extends Command {
    private final DriveSubsystem swerveSubsystem;
    private final Supplier<Double> xSpeedFunction, ySpeedFunction, turningSpeedFunction;
    private final Supplier<Boolean> fieldOrientedFunction;
    private final SlewRateLimiter xLimiter, yLimiter, turningLimiter;

    public SwerveJoystickCmd(DriveSubsystem swerveSubsystem, 
                            Supplier<Double> xSupplier, 
                            Supplier<Double> ySupplier, 
                            Supplier<Double> turningSupplier, 
                            Supplier<Boolean> fieldOrientedSupplier) {
        this.swerveSubsystem = swerveSubsystem;
        this.xSpeedFunction = xSupplier;
        this.ySpeedFunction = ySupplier;
        this.turningSpeedFunction = turningSupplier;
        this.fieldOrientedFunction = fieldOrientedSupplier;

        // 初始化加速度限制器
        this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        // 1. 取得搖桿原始輸入
        double xSpeed = xSpeedFunction.get();
        double ySpeed = ySpeedFunction.get();
        double turningSpeed = turningSpeedFunction.get();

        // 2. 套用死區 (Deadband) 處理
        xSpeed = Math.abs(xSpeed) > OIConstants.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > OIConstants.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > OIConstants.kDeadband ? turningSpeed : 0.0;

        // 3. 套用加速度限制並縮放至最大速度
        xSpeed = xLimiter.calculate(xSpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * DriveConstants.kTeleDriveMaxSpeedMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * DriveConstants.kTeleDriveMaxAngularSpeedRadiansPerSecond;
        
        // 4. 計算底盤速度 (ChassisSpeeds)
        ChassisSpeeds chassisSpeeds;
        if (fieldOrientedFunction.get()) {    
            // 場地相對坐標控制
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed, ySpeed, turningSpeed, swerveSubsystem.getRotation2d()
            );
        } else {
            // 機器人相對坐標控制
            chassisSpeeds = new ChassisSpeeds(xSpeed, ySpeed, turningSpeed);
        }

        // 5. 離散化處理：減少高速旋轉時的平移漂移
        chassisSpeeds = ChassisSpeeds.discretize(chassisSpeeds, 0.02);

        // 6. 輸出控制
        if (xSpeed == 0 && ySpeed == 0 && turningSpeed == 0) {
            swerveSubsystem.stopModules();
        } else {
            SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            swerveSubsystem.setModuleStates(moduleStates);
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules(); // 停止時確保馬達關閉
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}