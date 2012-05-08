#include "WPILib.h"
#include <Dashboard.h>
#include <DriverStationLCD.h>
#include <nivision.h>  //For image code
#include "Timer.h"
#include <vector>
#include <queue>
#include <algorithm>
#include <Vision/AxisCamera.h>
#include <Vision/RGBImage.h>
#include <math.h>
#include <string.h>
using namespace std;

int criteriaCount = 3;
ParticleFilterCriteria2 criteria[3];
ParticleFilterOptions options = {FALSE, FALSE, FALSE};

ShapeReport target;
bool freshTarget;

AxisCamera *cam;

float BS = 0.3;

double turnOffset = 0;
float susanSpeed = 0;
int boardHeight = 0;

int counts = 0;
int writtenImages = 0;

int imageTrack();
int imageProcessing();
int imageProcessingLoop();

void setTurnOffset( double newVal );
double getTurnOffset();
void setHeight( int newVal );
int getHeight();
void setSusanSpeed(float newVal);
float getSusanSpeed();

Task locateBackboard("imageTrackTask", (FUNCPTR) imageProcessingLoop);

DriverStationLCD *text;

template<typename T>
class lockedVariable {
private:
	T hidden;
public:
	lockedVariable() {}
	lockedVariable(T init) {
		hidden = init;
	}
	lockedVariable(const lockedVariable<T>& init) {
		hidden = init->get();
	}
	~lockedVariable() {
		delete hidden;
	}
	
	T get() {
		Synchronized s(semaphore);
		return hidden;
	}
	T set(T val) {
		Synchronized s(semaphore);
		return (hidden = val);
	}
};

class turnOffsetSource : public PIDSource {
public:
	turnOffsetSource();
	~turnOffsetSource();
	virtual double PIDGet();
};

class susanOutput : public PIDOutput {
public:
	susanOutput();
	virtual ~susanOutput();
	virtual void PIDWrite(float);
};

class IntegralMotorOutput : public PIDOutput {
private:
	SpeedController *motor;
	float mult;
public:
	IntegralMotorOutput(SpeedController *_motor, float _mult=-1) {
		motor = _motor;
		mult = _mult;
	}
	~IntegralMotorOutput() {}
	
	void PIDWrite(float output) {
		motor->Set(motor->Get() + mult * output);
		//printf("pid drive enabled");
	}
};

/*class FilteredEncoderInput : public Encoder {
private:
	Timer timer;
	double lastDistance;
public:
	FilteredEncoderInput(UINT32 aChannel, UINT32 bChannel, bool reverseDirection=false, EncodingType encodingType=k4X) {
		Encoder::Encoder(aChannel, bChannel, reverseDirection, encodingType);
		lastDistance = 0.0;
	}
	
	FilteredEncoderInput(UINT8 aModuleNumber, UINT32 aChannel, UINT8 bModuleNumber, UINT32 _bChannel, bool reverseDirection=false, EncodingType encodingType=k4X) {
		Encoder::Encoder(aModuleNumber, aChannel, bModuleNumber, _bChannel, reverseDirection, encodingType);
		lastDistance = 0.0;
	}
	
	~FilteredEncoderInput() {
		Encoder::~Encoder();
	}
	
	void Start() {
		Encoder::Start();
		timer.Start();
	}
	
	void Stop() {
		Encoder::Stop();
		timer.Stop();
	}
	
	double GetRate() {
		double dist = GetDistance();
		double rate = (dist - lastDistance) / timer.Get();
		lastDistance = dist;
		timer.Reset();
		return rate;
	}
};*/

class FilteredCounterInput : public Counter, public PIDSource {
private:
	Timer timer;
	int lastDistance;
public:
	FilteredCounterInput(UINT32 aChannel) :
		Counter::Counter(aChannel)
	{
		lastDistance = 0.0;
	}
	
	FilteredCounterInput(UINT8 aModuleNumber, UINT32 aChannel) :
		Counter::Counter(aModuleNumber, aChannel)
	{
		lastDistance = 0.0;
	}
	
	~FilteredCounterInput() {}
	
	void Start() {
		Counter::Start();
		timer.Start();
	}
	
	void Stop() {
		Counter::Stop();
		timer.Stop();
	}
	
	double GetRate() {
		int dist = Get();
		double rate = (double)(dist - lastDistance) / timer.Get();
		lastDistance = dist;
		timer.Reset();
		return rate;
	}
	
	double PIDGet() {
		return 1.0 / GetPeriod();
	}
};

class PIDSpeedController : public PIDController, public SpeedController {
public:
	float speed, scale;
	SpeedController *motor;
	IntegralMotorOutput motorOutput;
	
	PIDSpeedController(float p, float i, float d, PIDSource *source, SpeedController *_motor, float _scale) :
		motorOutput(_motor),
		speed(0.0),
		PIDController(p, i, d, source, &motorOutput)
	{
		SetOutputRange(-1.0, 1.0);
		scale = _scale;
		motor = _motor;
	}
	
	~PIDSpeedController() {
	}
	
	void Enable() {
		PIDController::Enable();
		Set(speed);
	}
	
	void Disable() {
		PIDController::Disable();
		Set(speed);
	}
	
	float Get() {
		return speed;
	}
	
	void Set(float _speed, UINT8 syncGroup=0) {
		speed = _speed;
		if (this->IsEnabled()) SetSetpoint(speed * scale);
		else motor->Set(speed);
		//motor->Set(speed);
	}
};


class RobotDemo : public SimpleRobot
{
	
	RobotDrive myRobot;

	Joystick stick1;
	Joystick gamePad;
	
	Relay wheelf;
	Relay wheelr;
	
	Relay conveyorf;
	Relay conveyorr;
	
	DigitalInput limit;
	DigitalInput *aimlimit;
	
	Victor *susan;
	Victor *vex;
	
	Victor launch1;
	Victor launch2;
	
	DriverStation *drives;
	Dashboard *dash;
	
	PIDController *susanpid;
	turnOffsetSource *source;
	susanOutput *output;
	
	bool stick1ToggleStates [13];
	bool stick1OldButtonStates [13];
	bool stick1NewButtonStates [13];

	bool gamePadToggleStates [11];
	bool gamePadOldButtonStates [11];
	bool gamePadNewButtonStates [11];
	
	float throttle;
	float launchThrottle;
	
	float autoLaunchSpeed;
	float autoLaunchSetSpeed;

	Timer time;
	
	float dist;
	
	//float susanSpeed;
	int susanDirection;
	int badSusanDirection;
	bool susanLimitOld;
	
	int i;
	int conveyorDirection;

	Encoder flEncoder;
	Encoder frEncoder;
	Encoder blEncoder;
	Encoder brEncoder;
	
	Jaguar flMotor;
	Jaguar frMotor;
	Jaguar blMotor;
	Jaguar brMotor;
	
	PIDSpeedController flSpeed;
	PIDSpeedController frSpeed;
	PIDSpeedController blSpeed;
	PIDSpeedController brSpeed;
	
	//PIDController launch1pid;
	//PIDController launch2pid;
	FilteredCounterInput launch1Proximity;
	FilteredCounterInput launch2Proximity;
	PIDController launch1pid;
	
	float distTable [15];
	float powerTable [15];
	int dataPoints;
	
	float P, I, D, maxRPS;
	
	SmartDashboard *sdash;
	
public:
	RobotDemo(void):
		flMotor(1),
		blMotor(2),
		frMotor(3),
		brMotor(4),
		drives(),
		dash(),
		limit(1),
		wheelf(1,Relay::kForwardOnly),
		wheelr(2,Relay::kReverseOnly),
		conveyorf(3,Relay::kForwardOnly),
		conveyorr(4,Relay::kReverseOnly),
		stick1(1),
		gamePad(2),
		launch1(5),
		launch2(6),
		flEncoder(7, 8),
		frEncoder(9, 10),
		blEncoder(3, 4),
		brEncoder(5, 6),
		dataPoints(15),
		//PID Constants for drive train///////////////////////////////////////////////////////////////////////////////////////
		P(0.3),
		I(0.06),
		D(0.07),
		maxRPS(2500.0),
		
		flSpeed(P, I, D, &flEncoder, &flMotor, maxRPS),
		frSpeed(P, I, D, &frEncoder, &frMotor, maxRPS),
		blSpeed(P, I, D, &blEncoder, &blMotor, maxRPS),
		brSpeed(P, I, D, &brEncoder, &brMotor, maxRPS),
		
		myRobot(&flSpeed, &blSpeed, &frSpeed, &brSpeed),
		
		launch1Proximity(11),
		launch2Proximity(12),
		launch1pid(0.05 / 166.0, 0.000 / 166.0, 0.02 / 166.0, &launch1Proximity, new IntegralMotorOutput(&launch1, -1))
	{
		dist = 0;
		
		susanSpeed = 0;
		susanDirection = 0;
		badSusanDirection = 0;
		susanLimitOld = false;
		
		i = 1;
		conveyorDirection = 0;
		
		//dash = new Dashboard();
		//myRobot.SetExpiration(0.1);
		text = DriverStationLCD::GetInstance();
		cam = &AxisCamera::GetInstance("10.33.24.11");
		cam->WriteResolution(AxisCamera::kResolution_320x240);
		cam->WriteMaxFPS(6);

		susan = new Victor(7);
		launchThrottle = 0.0;

		vex = new Victor(8);
		
		sdash = SmartDashboard::GetInstance();
		
		autoLaunchSpeed = 0.0;
		autoLaunchSetSpeed = 0.0;
		
		source = new turnOffsetSource();
		aimlimit = new DigitalInput(2);
		output = new susanOutput(); 
		
		//practice robot, which is less efficient in transmitting energy
		//susanpid = new PIDController(0.275 / 160.0, 0.02 / 160.0, 0.37 / 160.0, source, output, 0.2);
		//competition robot, more efficient energy transfer
		susanpid = new PIDController(0.2 / 160.0, 0.015 / 160.0, 0.27 / 160.0, source, output, 0.2);
		susanpid->SetOutputRange(-1.0, 1.0);
		
		flEncoder.Start();
		frEncoder.Start();
		blEncoder.Start();
		brEncoder.Start();
		flEncoder.SetPIDSourceParameter(Encoder::kRate);
		frEncoder.SetPIDSourceParameter(Encoder::kRate);
		blEncoder.SetPIDSourceParameter(Encoder::kRate);
		brEncoder.SetPIDSourceParameter(Encoder::kRate);
		
		/*flSpeed.Enable();
		frSpeed.Enable();
		blSpeed.Enable();
		brSpeed.Enable();*/
		
		myRobot.SetInvertedMotor(RobotDrive::kFrontLeftMotor, true);
		myRobot.SetInvertedMotor(RobotDrive::kRearLeftMotor, true);
		myRobot.SetInvertedMotor(RobotDrive::kFrontRightMotor, true);
		myRobot.SetInvertedMotor(RobotDrive::kRearRightMotor, true);
		
		//Used in imageProcessing Task
		criteria[0].parameter = IMAQ_MT_AREA;
		criteria[0].lower = 128;
		criteria[0].upper = 100000000;
		criteria[0].calibrated = FALSE;
		criteria[0].exclude = FALSE;

		criteria[1].parameter = IMAQ_MT_NUMBER_OF_HOLES;
		criteria[1].lower = 1;
		criteria[1].upper = 2;
		criteria[1].calibrated = FALSE;
		criteria[1].exclude = FALSE;
		
		criteria[2].parameter = IMAQ_MT_AREA_BY_PARTICLE_AND_HOLES_AREA;
		criteria[2].lower = 0.5;
		criteria[2].upper = 100;
		criteria[2].calibrated = FALSE;
		criteria[2].exclude = FALSE;
				
		freshTarget = false;
		throttle = 0.8;
		
		memset(stick1OldButtonStates, 0, 12);
		memset(stick1ToggleStates, 0, 12);
		memset(gamePadOldButtonStates, 0, 11);
		memset(gamePadToggleStates, 0, 11);
		
		//Fill in dist Table with values of the distances corresponding to motor power gained from testing

		distTable[0] = 0;
		distTable[1] = 7;
		distTable[2] = 8;
		distTable[3] = 9;
		distTable[4] = 10;
		distTable[5] = 11;
		distTable[6] = 12;
		distTable[7] = 13;
		distTable[8] = 14;
		distTable[9] = 15;
		distTable[10] = 16;
		distTable[11] = 17;
		distTable[12] = 18;
		distTable[13] = 19;
		distTable[14] = 20;

		//Fill in power Table with values of the powers ( 0.0 - 1.0) corresponding to distance traveled gained from testing

		powerTable[0] = 0;
		powerTable[1] = 3.71;
		//powerTable[2] = 3.71;
		powerTable[2] = 3.87;
		powerTable[3] = 4.04;
		powerTable[4] = 4.23;
		powerTable[5] = 4.27;
		powerTable[6] = 4.42;
		powerTable[7] = 4.46;
		powerTable[8] = 4.55;
		powerTable[9] = 4.73;
		powerTable[10] = 4.79;
		powerTable[11] = 4.89;
		powerTable[12] = 5.01;
		//powerTable[13] = 4.89;
		powerTable[13] = 5.33;
		powerTable[14] = 5.65;
	}
	
	void tank(float left, float right) {
		flSpeed.Set(-left);
		//blSpeed.Set(-left);
		frSpeed.Set(right);
		//brSpeed.Set(right);
		
		/*
		flMotor.Set(left);
		blMotor.Set(left);
		frMotor.Set(-right);
		brMotor.Set(-right);
		*/
	}
	
	void Autonomous(void)
	{
		Timer t;
		int step = 0;
		myRobot.SetSafetyEnabled(false);
		
		//Value that indicates error in Image Tracking
		setTurnOffset(0);
		setHeight(0);
		locateBackboard.Start();
		susanpid->SetSetpoint(0.0);
		//if (!susanpid->IsEnabled()) susanpid->Enable();
		float launchSpeed;
		t.Start();
		//myRobot.TankDrive(1.0, 1.0);
		while( IsAutonomous() && !IsDisabled() )
		{
			//Sets the error of imageProcessing
			//Creates a critical Section to Protect turnOffset
			double error = -getTurnOffset();
			//vex->Set(0.5);
			//myRobot.TankDrive(0.5, 0.5);
			//double ratio = getWidth() / 320;
			//dist = 12.0 / tan(3.14159265358979323846264338 * 23.5 / 180.0) / ratio;
			//dist = (12 / tan(3.14159265 * 23.5 / 180)) / ((float)getHeight() / 320);
			dist = 580.0 / (float)getHeight();
			dist = ((dist >= 12.0 && dist <= 20.0 ) ? dist : 19.0);
			
			//Moves the turret
			//moveSusan(false, susanSpeed);
			
			/*if (step == 0 && t.Get() >= 0.0) {
				//launchSpeed = scalePower( distToNormalizedPower( dist ) 2.70, DriverStation::GetInstance()->GetBatteryVoltage());
				//launchSpeed = max(0.16, min(launchSpeed, 0.25));
				//launchSpeed = 0.22;
				susanpid->Disable();
				step++;
				susanSpeed = 0.0;

				float backSpin = (true) ? BS : 0;
				//launchSpeed = scalePower( distToNormalizedPower( dist ), DriverStation::GetInstance()->GetBatteryVoltage());
				//launchSpeed = max(0.2, min(launchSpeed, 0.45));
				//launchSpeed = 0.217;
				//Actuates the launchers based on launchSpeed
				launch1.Set(-launchSpeed - backSpin);
				launch2.Set(-launchSpeed + backSpin);
				wheelf.Set(Relay::kOn);
				t.Reset();
			} else if (step == 1 && t.Get() >= 0.3) {
				step++;
				//launchSpeed = scalePower( 2.70, DriverStation::GetInstance()->GetBatteryVoltage());
				//launchSpeed = max(0.16, min(launchSpeed, 0.25));
				launchSpeed = 0.205;
				wheelf.Set(Relay::kOff);
				float backSpin = BS;
				launch1.Set(-launchSpeed * (1 - backSpin) );
				launch2.Set(-launchSpeed * (1 + backSpin) );
				t.Reset();
			} else if (step == 2 && t.Get() >= 5.0) {
				step++;
				conveyorf.Set(Relay::kOn);
				t.Reset();
			} else if (step == 3 && t.Get() >= 1.0) {
				step++;
				conveyorf.Set(Relay::kOff);
				t.Reset();
			} else if (step == 4 && t.Get() >= 6.5) {
				step++;
				conveyorf.Set(Relay::kOn);
				t.Reset();
			} else if (step == 5 && t.Get() >= 15.0) {
				step++;
				conveyorf.Set(Relay::kOff);
				launch1.Set(0.0);
				launch2.Set(0.0);
				//wheelf.Set(Relay::kOn);
				t.Reset();
			} */
			
			//2 point shot
			/*if (step == 0 && t.Get() >= 0.0) {
				step++;
				//launchSpeed = scalePower( 2.70, DriverStation::GetInstance()->GetBatteryVoltage());
				//launchSpeed = max(0.16, min(launchSpeed, 0.25));
				launchSpeed = 0.1625;
				float backSpin = BS;
				launch1.Set(-launchSpeed * (1 - backSpin) );
				launch2.Set(-launchSpeed * (1 + backSpin) );
				t.Reset();
				
				myRobot.ArcadeDrive(0.5, 0.0);
				
			} else if (step == 1 && t.Get() >= 5.0) {
				step++;
				myRobot.ArcadeDrive(0.0,0.0);
				wheelf.Set(Relay::kOn);
				t.Reset();
			} else if (step == 2 && t.Get() >= 0.3) {
				step++;
				wheelf.Set(Relay::kOff);
				//conveyorf.Set(Relay::kOn);
				t.Reset();
			} else if (step == 3 && t.Get() >= 1.0) {
				step++;
				conveyorf.Set(Relay::kOn);
				t.Reset();
			}else if (step == 4 && t.Get() >= 1.0) {
				step++;
				conveyorf.Set(Relay::kOff);
				t.Reset();
			} else if (step == 5 && t.Get() >= 4.5) {
				step++;
				conveyorf.Set(Relay::kOn);
				t.Reset();
			} else if (step == 6 && t.Get() >= 15.0) {
				step++;
				conveyorf.Set(Relay::kOff);
				launch1.Set(0.0);
				launch2.Set(0.0);
				//wheelf.Set(Relay::kOn);
				t.Reset();
			}*/
			
			
			/*else if (step == 6 && t.Get() >= 0.5) {
				step++;
				wheelf.Set(Relay::kOff);
				myRobot.ArcadeDrive(0, 0.9, false);
				t.Reset();
			} else if (step == 7 && t.Get() >= 1.8) {
				step++;
				myRobot.ArcadeDrive(0, 0, false);
				t.Reset();
			} else if (step == 8 && t.Get() >= 0.3) {
				step++;
				myRobot.ArcadeDrive(0.7, 0, false);
				t.Reset();
			} else if (step == 9 && t.Get() >= 2.333333) {
				step++;
				myRobot.ArcadeDrive(0, 0, false);
				t.Reset();
			}*/
				
			
			//Updates Dashboard for feedback
			text->Clear();
			text->Printf(DriverStationLCD::kUser_Line1, 1, "%s", "Team 3324 Autonomous" );
			text->Printf(DriverStationLCD::kUser_Line2, 1, "Power: %f", launchSpeed );
			text->Printf(DriverStationLCD::kUser_Line3, 1, "Track: %f", getTurnOffset() );
			text->Printf(DriverStationLCD::kUser_Line4, 1, "dist: %f", getHeight() );
			text->Printf(DriverStationLCD::kUser_Line5, 1, "step: %d, time: %f", step, t.Get());
			sdash->PutInt("flencoder", flEncoder.Get());
			sdash->PutInt("frencoder", frEncoder.Get());
			sdash->PutInt("blencoder", blEncoder.Get());
			sdash->PutInt("brencoder", brEncoder.Get());
			text->Printf(DriverStationLCD::kUser_Line6, 1, "encoders: %d %d %d %d\n", flEncoder.Get(), frEncoder.Get(), blEncoder.Get(), brEncoder.Get());
			text->UpdateLCD();
		}
			
			
	}
	
	//Handles toggleStates for both gamePad and JoyStick
	void handleToggles()
	{
		bool oldLaunchThrottleDown = stick1ToggleStates[7];
		bool oldLaunchThrottleUp = stick1ToggleStates[6];
		for( int i = 1;i<=11;i++){
			stick1NewButtonStates[i] = stick1.GetRawButton( i );
			
			if(!stick1OldButtonStates[i] && stick1NewButtonStates[i] ){
				stick1ToggleStates[i] = !stick1ToggleStates[i];
			}
			
			stick1OldButtonStates[i] = stick1NewButtonStates[i];
		}

		bool oldThrottleDown = gamePadToggleStates[5];
		bool oldThrottleUp = gamePadToggleStates[6];
		
		for( int i = 1;i<=10;i++){
			gamePadNewButtonStates[i] = gamePad.GetRawButton( i );
			
			if(!gamePadOldButtonStates[i] && gamePadNewButtonStates[i] ){
				gamePadToggleStates[i] = !gamePadToggleStates[i];
			}
			
			gamePadOldButtonStates[i] = gamePadNewButtonStates[i];
		}
		
		if(oldThrottleUp != gamePadToggleStates[6]){
			throttle = throttle + 0.1;
		}
		if(oldThrottleDown != gamePadToggleStates[5]){
			throttle = throttle - 0.1;
		}
		
		if (oldLaunchThrottleUp != stick1ToggleStates[6]) {
			launchThrottle += 0.01;
		}
		if (oldLaunchThrottleDown != stick1ToggleStates[7]) {
			launchThrottle -= 0.01;
		}
		
		throttle = min(max(throttle, 0.5), 1);
		launchThrottle = max(min(launchThrottle, 1), 0);
		
	}
	
	//Moves the lazy Susan based on global variable susanSpeed
	//Takes into account the limit switch and an override for the limit switch
	void moveSusan( bool override, float susanSpeed ){
		
		susanDirection = (susanSpeed > 0)? 1: ( (susanSpeed < 0)? -1:0 );
					
		if( override ){
			vex->Set(susanSpeed);
		}
		else{
			if( aimlimit->Get() && !susanLimitOld ){
				badSusanDirection = susanDirection;
			}
			if( !aimlimit->Get() ){
				badSusanDirection = 0;
			}
	
			if( badSusanDirection != susanDirection ){
				vex->Set( susanSpeed );
			}
			else{
				vex->Set(0);
			}
		}
		susanLimitOld = aimlimit->Get();
		
	}
	
	
	
	float distToNormalizedPower( float distance ){
		int under = 0;
		int over = 1;

		for( int i = 0; i < dataPoints; i++ ){
			if( distance >= distTable[i] ){
				under = i - 1;
				over = i;
			}
		}

		return powerTable[under] + (distance - distTable[under])*( powerTable[over] - powerTable[under] )/( distTable[over] - distTable[under] );

	}

	float scalePower( float power, float voltage ){
		return power / voltage;
	}
	
	void OperatorControl(void)
	{
		launch1Proximity.Start();
		launch2Proximity.Start();
		launch1pid.SetSetpoint(20.0);
		locateBackboard.Start();
		susanpid->SetSetpoint(0.0);
		bool oldTrackToggle = false;
		bool trackerEnabled = false;

		bool oldAutoSpeedToggle = false;
		bool autoLaunchEnabled = false;
		
		//launch1pid.Enable();
		
		while (IsOperatorControl() && ! IsDisabled())
		{
			
			//Handles the Toggles and Throttle
			handleToggles();
			
			if( gamePadToggleStates[8] != oldTrackToggle ){
				if (susanpid->IsEnabled()) susanpid->Disable();
				else susanpid->Enable();
				trackerEnabled = gamePadToggleStates[8];
			}
			oldTrackToggle = gamePadToggleStates[8];
				
			//Sets the belt direction
			conveyorDirection = gamePad.GetRawButton(2) ? 2 : ( gamePadToggleStates[1] ? 1 : 0 );
			conveyorDirection = stick1.GetRawButton(5) ? 0 : ( stick1.GetRawButton(4) ? 1 : ( stick1.GetRawButton(2) ? 2 : conveyorDirection ) );

			//Actuates the belt based on conveyorDirection
			conveyorf.Set( (conveyorDirection == 1) ? Relay::kOn : Relay::kOff );
			conveyorr.Set( (conveyorDirection == 2) ? Relay::kOn : Relay::kOff );
			
			//Gets the throttle set in handleToggles
			float mult = throttle;
			
			//Sets reversable front end driving
			i = gamePadToggleStates[3] ? 1 : -1;

			//Drives based on arcade/tank state, normal/reversed state, and the throttle multiplier
			if( gamePadToggleStates[4] ){
				myRobot.ArcadeDrive( i*mult*gamePad.GetRawAxis(2), -mult*gamePad.GetRawAxis(1) );
			}
			else {
				if( i == -1 ) {
					myRobot.TankDrive(-mult*gamePad.GetRawAxis(2),-mult*gamePad.GetRawAxis(5));
					//tank(-mult*gamePad.GetRawAxis(2),-mult*gamePad.GetRawAxis(5));
				}
				else {
					myRobot.TankDrive(mult*gamePad.GetRawAxis(5),mult*gamePad.GetRawAxis(2));
					//tank(mult*gamePad.GetRawAxis(5),mult*gamePad.GetRawAxis(2));
				}
			}
			
			//Actuates the arm, taking into account the limit switch
			wheelf.Set( stick1.GetRawButton(8) ? Relay::kOn : Relay:: kOff );
			wheelr.Set( stick1.GetRawButton(9) /*&& !limit.Get()*/ ? Relay::kOn : Relay::kOff );
			
			//Sets susanSpeed and moves the susan with optional override
			if (trackerEnabled) {
				moveSusan(false, getSusanSpeed());
			} else {
				moveSusan( stick1.GetRawButton(3), -0.5 * stick1.GetX());
			}

			dist = 580.0 / (float)getHeight();
			autoLaunchSpeed = scalePower( distToNormalizedPower( dist + 1 ), DriverStation::GetInstance()->GetBatteryVoltage() );

			/*if( stick1ToggleStates[11] != oldAutoSpeedToggle ){
				if( stick1ToggleStates[11] ){
					autoLaunchSetSpeed = autoLaunchSpeed;
					autoLaunchEnabled = true;
				}
				else{
					autoLaunchSetSpeed = 0;
					autoLaunchEnabled = false;
				}
			}*/

			float launchSpeed = (-stick1.GetThrottle() + 1)/2 + launchThrottle;
			
			if( stick1ToggleStates[10] ){
				//launchSpeed = scalePower( 11.6 * 0.159, DriverStation::GetInstance()->GetBatteryVoltage() );
				launchSpeed = 0.165	;
			}
			
			if( stick1ToggleStates[11] ){
				launchSpeed = scalePower( 0.212 * 12.4, DriverStation::GetInstance()->GetBatteryVoltage() );
			}

			float backSpin = BS;
			
			launch1.Set( stick1ToggleStates[1] ? -( ( autoLaunchEnabled )? autoLaunchSetSpeed : launchSpeed ) * (1 - backSpin) : 0 );
			launch2.Set( stick1ToggleStates[1] ? -( ( autoLaunchEnabled )? autoLaunchSetSpeed : launchSpeed ) * (1 + backSpin) : 0 );

			oldAutoSpeedToggle = stick1ToggleStates[11];			

			if(gamePad.GetRawButton(7)){
				launch1.Set(0.5);
				launch2.Set(0.5);
			}
			
			
			
			//Prints to Dashboard for feedback
			text->Clear();
			text->Printf(DriverStationLCD::kUser_Line1, 1, "%s %s %f", (gamePadToggleStates[4])? "Arcade" : "Tank", (i == -1)? "" : "Reverse", mult );
			text->Printf(DriverStationLCD::kUser_Line2, 1, "Belt: %s", (conveyorDirection==0)?"Off": ( (conveyorDirection==1)? "Forward":"Backward" ) );
			text->Printf(DriverStationLCD::kUser_Line3, 1, "Launcher: %s %f %s %s", (stick1ToggleStates[1])? "On": "Off", launchSpeed, (gamePadToggleStates[7])?"":"BackSpin", (stick1ToggleStates[10])? "Fender":"", (stick1ToggleStates[11])? "Key":"");
			text->Printf(DriverStationLCD::kUser_Line4, 1, "%s", (stick1ToggleStates[10]? "Fender":" "));			
			//text->Printf(DriverStationLCD::kUser_Line5, 1, "AutoPower: %s %f", (stick1ToggleStates[11])?"Set: ":"Off: ", (autoLaunchEnabled)? autoLaunchSetSpeed : autoLaunchSpeed );
			//text->Printf(DriverStationLCD::kUser_Line6, 1, "Tracking: %s: %f, %f %f", (trackerEnabled) ? "On" : "Off", getTurnOffset(), dist, DriverStation::GetInstance()->GetBatteryVoltage() );
			text->Printf(DriverStationLCD::kUser_Line6, 1, "%f, %f, pid error: %f\n", launch1Proximity.PIDGet(), launch2Proximity.PIDGet(), launch1pid.GetError() );
			//text->Printf(DriverStationLCD::kUser_Line6, 1, "%d, %d\n", launch1Proximity.Get(), launch2Proximity.Get() );
			text->UpdateLCD();
			
			Wait(0.005);
			
		}
	}
	
	void Disabled(void)
	{
		locateBackboard.Start();
		if (susanpid->IsEnabled()) {
			susanpid->Disable();
			susanpid->Reset();
		}
		if (launch1pid.IsEnabled()) launch1pid.Disable();
		
		//Sets dashboard text
		text->Clear();
		text->Printf(DriverStationLCD::kUser_Line1, 1, "Disabled");
		text->UpdateLCD();
	}
};

bool ycomp(pair<double, double> a, pair<double, double> b) {
	return a.second < b.second; 
}

pair<double, double> trackableCoordinates(vector<pair<double, double> > centroids) {
	pair<double, double> ret;
	
	if (centroids.size() == 1) {
		ret = centroids[0];
	} else if (centroids.size() == 2) {
		sort(centroids.begin(), centroids.end(), ycomp);
		ret = centroids[0];
	} else if (centroids.size() == 3) {
		sort(centroids.begin(), centroids.end());
		ret = centroids[centroids.size() / 2];
	} else if (centroids.size() == 4) {
		sort(centroids.begin(), centroids.end(), ycomp);
		ret = centroids[0];
	}
	
	return ret;
}

int imageProcessing() {
	if(cam->IsFreshImage()) {
		Timer t;
		t.Start();
		int numParticles = 0;
		RGBImage *img;
		cam->GetImage( img );
		if (counts % 25 == 0 && !RobotDemo::getInstance().IsDisabled() && writtenImages < 20) {
			char s[128];
			sprintf(s, "img%d.jpg", counts / 25);
			img->Write(s);
			writtenImages++;
		}
		//inside build space tables/build space practice field
		//BinaryImage *b = img->ThresholdHSL( 64, 192, 60, 250, 60, 160 );
		//window in build space
		//BinaryImage *b = img->ThresholdHSL(32, 248, 32, 250, 96, 224); 
		//Buckeye competition field
		//BinaryImage *b = img->ThresholdHSL(30, 128, 64, 255, 128, 224);
		//Buckeye competition field (max shutter=1/5000s)
		
		//BinaryImage *b = img->ThresholdRGB( 10, 117, 0, 71, 41, 255 );
		
		//BinaryImage *b = img->ThresholdHSL(0, 255, 0, 128, 8, 64);
		
		//Buckeye practice field (max shutter time=1/250s)
		//BinaryImage *b = img->ThresholdHSL(64, 224, 64, 255, 128, 224);
		//cinci competition, max shutter = 1/2000s
		BinaryImage *b = img->ThresholdHSL(0, 255, 0, 196, 8, 72);
		
		Image *t1 = b->GetImaqImage();
		Image *t2 = imaqCreateImage( IMAQ_IMAGE_U8, 7 );
		
		imaqParticleFilter3(t2, t1, criteria, criteriaCount, &options, NULL, &numParticles);
		imaqCountParticles(t2, FALSE, &numParticles);
		
		double centerx = 160.0;
		double height;
		if (numParticles > 0) {
			//imaqMeasureParticle(t2, 0, 0, IMAQ_MT_CENTER_OF_MASS_X, &centerx);
			//imaqMeasureParticle(t2, 0, 0, IMAQ_MT_BOUNDING_RECT_HEIGHT, &height);
			
			//track multiple targets;
			vector<pair<double, double> > targetSizes;
			vector<pair<double, double> > targetCoords;
			for (int i = 0; i < numParticles; i++) {
				double centerx, centery;
				double width, height;

				imaqMeasureParticle(t2, i, 0, IMAQ_MT_CENTER_OF_MASS_X, &centerx);
				imaqMeasureParticle(t2, i, 0, IMAQ_MT_CENTER_OF_MASS_Y, &centery);
				imaqMeasureParticle(t2, i, 0, IMAQ_MT_BOUNDING_RECT_HEIGHT, &height);
				imaqMeasureParticle(t2, i, 0, IMAQ_MT_BOUNDING_RECT_WIDTH, &width);
				targetSizes.push_back(make_pair(width, height));
				targetCoords.push_back(make_pair(centerx, centery));
			}
			
			//Creates a critical Section to Protect turnOffset
			//setTurnOffset( centerx - 160.0 );
			setTurnOffset(trackableCoordinates(targetCoords).first - 160.0);
			setHeight(height);
		}
		printf("centerx: %f, numParticles: %d\n", centerx, numParticles);
		
		delete img;
		delete b;
		imaqDispose(t1);
		imaqDispose(t2);
		
		counts++;
		t.Stop();
		printf("fps: %f\n", 1.0 / t.Get());
	} else {
		printf("no image\n");
	}
}

int imageProcessingLoop() {
	while (true) {
		imageProcessing();
		Wait(0.005);
	}
	return 0;
}

void setTurnOffset( double newVal ){
	Synchronized s(semaphore);
	turnOffset = newVal;
}

double getTurnOffset(){
	Synchronized s(semaphore);
	return turnOffset;
}

void setHeight( int newVal ){
	Synchronized s(semaphore);
	boardHeight = newVal;
}

int getHeight(){
	Synchronized s(semaphore);
	return boardHeight;
}

void setSusanSpeed( float newVal ){
	Synchronized s(semaphore);
	susanSpeed = newVal;
}

float getSusanSpeed(){
	Synchronized s(semaphore);
	return susanSpeed;
}

turnOffsetSource::turnOffsetSource() {}
turnOffsetSource::~turnOffsetSource() {}

double turnOffsetSource::PIDGet() {
	return getTurnOffset();
}

susanOutput::susanOutput() {}
susanOutput::~susanOutput() {}

void susanOutput::PIDWrite(float output) {
	setSusanSpeed(output);
	//setSusanSpeed(getSusanSpeed() - offset());
}

START_ROBOT_CLASS(RobotDemo);
