package org.jmhsrobotics.frc2023.util.DrivePorts;

import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.Map;

import org.jmhsrobotics.frc2023.Constants.RobotType;
import org.jmhsrobotics.frc2023.util.IIMUWrapper;
import org.jmhsrobotics.frc2023.util.NavxIMU;
import org.jmhsrobotics.frc2023.util.PIDConfig;
import org.jmhsrobotics.frc2023.util.PigeonIMU;
import org.jmhsrobotics.frc2023.util.ProfiledPIDConfig;
import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import com.ctre.phoenix.sensors.Pigeon2Configuration;

import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.SPI;

public class DrivePortsObject {

	private JSONObject parsedJSONObject;

	private ProfiledPIDConfig autoDistanceProfiledPID;
	private PIDConfig keepHeadingPID;
	private PIDConfig balancingPID;

	private IIMUWrapper imu;

	// spotless:off
    private static final Map<RobotType, String> layoutFilenames = Map.of(
        RobotType.SUSAN,    "DrivePortsLayouts/SusanDrivePorts.json",
        RobotType.BABY_BOT, "DrivePortsLayouts/SusanDrivePorts.json",
        RobotType.BOT_2020, "DrivePortsLayouts/SusanDrivePorts.json"
    );
    // spotless:on

	public DrivePortsObject(RobotType robot) {

		try {

			this.parseJSON(layoutFilenames.get(robot));

		} catch (IOException e) {

			DataLogManager.log("Failed to read driveports layout JSON (default to BabyBot).");
			System.out.println("ERROR: Failed to read driveports layout JSON.");
			System.err.println("WARNING!: DEFAULTED TO BABYBOT");

			try {

				this.parseJSON(layoutFilenames.get(RobotType.BABY_BOT));

			} catch (IOException eFatal) {

				DataLogManager.log("FATAL: Failed to read defaulted BabyBot layout JSON.");
				System.out.println("FATAL ERROR: Failed to read the defaulted BabyBot layout JSON.");

				return;
			}
		}

		this.autoDistanceProfiledPID = this.parseProfiledPIDConfig("autoDistanceProfiledPID");

		this.keepHeadingPID = this.parsePIDConfig("keepHeadingPID");
		this.balancingPID = this.parsePIDConfig("balancingPID");

		this.imu = this.parseIMU("imu");
	}

	public ProfiledPIDConfig getAutoDistanceProfiledPID() {
		// TODO Auto-generated method stub
		return this.autoDistanceProfiledPID;
	}

	public PIDConfig getKeepHeadingPID() {
		// TODO Auto-generated method stub
		return this.keepHeadingPID;
	}

	public PIDConfig getBalancingPID() {
		// TODO Auto-generated method stub
		return this.balancingPID;
	}

	public int getLeftFrontMotorCANId() {
		return this.parsedJSONObject.getInt("leftFrontMotorCANId");
	}

	public int getRightFrontMotorCANId() {
		return this.parsedJSONObject.getInt("rightFrontMotorCANId");
	}

	public int getLeftRearMotorCANId() {
		return this.parsedJSONObject.getInt("leftRearMotorCANId");
	}

	public int getRightRearMotorCANId() {
		return this.parsedJSONObject.getInt("rightRearMotorCANId");
	}

	public boolean getRightFrontMotorInversion() {
		return this.parsedJSONObject.getBoolean("rightFrontMotorInversion");
	}

	public boolean getLeftFrontMotorInversion() {
		return this.parsedJSONObject.getBoolean("leftFrontMotorInversion");
	}

	public double getWheelDiameterInInches() {
		return this.parsedJSONObject.getDouble("wheelDiameterInInches");
	}

	public double getMaxVelocity() {
		// TODO Auto-generated method stub
		return this.parsedJSONObject.getDouble("maxVelocity");
	}

	public double getMaxAcceleration() {
		// TODO Auto-generated method stub
		return this.parsedJSONObject.getDouble("maxAcceleration");
	}

	public double getBalanceCreepSpeed() {
		// TODO Auto-generated method stub
		return this.parsedJSONObject.getDouble("balanceCreepSpeed");
	}

	public IIMUWrapper getIMU() {
		return imu;
	}

	private void parseJSON(String path) throws IOException {

		File jsonFile = new File(path);
		JSONTokener tokener = new JSONTokener(new FileInputStream(jsonFile));
		this.parsedJSONObject = new JSONObject(tokener);
	}

	private PIDConfig parsePIDConfig(String key) {

		JSONArray tempArr = this.parsedJSONObject.getJSONArray(key);

		// spotless:off
        PIDConfig config = new PIDConfig(
            tempArr.getDouble(0),
            tempArr.getDouble(1),
            tempArr.getDouble(2)
        );
        // spotless:on

		return config;
	}

	private ProfiledPIDConfig parseProfiledPIDConfig(String key) {

		JSONArray tempArr = this.parsedJSONObject.getJSONArray(key);

		// spotless:off
        ProfiledPIDConfig config = new ProfiledPIDConfig(
            tempArr.getDouble(0),
            tempArr.getDouble(1),
            tempArr.getDouble(2),
            new Constraints(
                tempArr.getDouble(3),
                tempArr.getDouble(4)
            )
        );
        // spotless:on

		return config;
	}

	private IIMUWrapper parseIMU(String key) {

		// spotless:off
        switch(this.parsedJSONObject.getString("imu")) {
            case "NAVX":
                return new NavxIMU(SPI.Port.kMXP);
            case "PIGEON":
                Pigeon2Configuration pigeonConfig = new Pigeon2Configuration();
		        pigeonConfig.EnableCompass = false;
                return new PigeonIMU(30, pigeonConfig);
            default:
                DataLogManager.log("Failed to define IMU (defaulted to NAVX).");
                System.out.println("WARNING: Failed to define IMU: defaulted to NAVX.");
                return new NavxIMU(SPI.Port.kMXP);
        }
        // spotless:on
	}
}
