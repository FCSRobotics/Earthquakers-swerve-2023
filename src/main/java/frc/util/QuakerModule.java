package frc.util;
// import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.util.QuakerSDSWrapper.QuakerSwerveHelper;
import frc.util.QuakerSDSWrapper.QuakerSwerveHelper.GearRatio;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class QuakerModule implements SwerveModule{
    SwerveModule module;
    double distance;
    int id;
    // private ShuffleboardTab mainTab;
    public QuakerModule(GearRatio gearRatio, int driveId, int steerId, int steerEncoder, double steerOffset, String name, int moduleID){
        module = QuakerSwerveHelper.createNeo( 
            Shuffleboard.getTab("Drivetrain").getLayout(name, BuiltInLayouts.kList)
                .withSize(2,4)
                .withPosition(id*2,0),           
            // This can either be STANDARD or FAST depending on your gear configuration
            gearRatio,
            driveId,
            steerId,
            steerEncoder,
            // This is how much the steer encoder is offset from true zero (In our case, zero is facing straight forward)
            steerOffset
        );
        distance = 0;
        id = moduleID;
    }
    // Inherited from Swerve Module 
    public double getDriveVelocity(){
        return module.getDriveVelocity();
    };
    public double getSteerAngle(){
        return module.getSteerAngle();
    };
    public void set(double driveVoltage, double steerAngle){
        module.set(driveVoltage, steerAngle);
    };
    public void update(){
        // distance = integral of velocity
        distance += getDriveVelocity() * 0.01;
    }
    public void reset(){
        distance = 0;
    }
    public int getID(){
        return id;
    }
    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(distance, new Rotation2d(getSteerAngle()));
    }

    
}
