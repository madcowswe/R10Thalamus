#include "thal.h"
#include <math.h>

// *** Config stuff
#define FIRMWARE_VERSION    1           // Firmware version
#define MESSAGE_LOOP_HZ     25          // Max frequency of messages in Hz (keep this number low, like around 25)
#define RX_PANIC            2           // Number of seconds after missing RX before craft considered "disconnected"

// *** LED stuff
unsigned char flashPLED, flashVLED, flashRLED;

// *** ILink stuff
ilink_identify_t ilink_identify;
ilink_thalstat_t ilink_thalstat;
ilink_rawimu_t ilink_rawimu;
ilink_attitude_t ilink_attitude;
ilink_thalparam_t ilink_thalparam_tx;
ilink_thalparam_t ilink_thalparam_rx;
ilink_thalpareq_t ilink_thalpareq;
ilink_iochan_t ilink_inputs0;
ilink_iochan_t ilink_outputs0;

void LinkInit(void);

// *** Timers and counters
unsigned int sysMS;
unsigned long long sysUS;
unsigned short RxWatchdog;

// *** Functions
void Calibrate(void);

// *** Configs
#define AHRS_RATE           800
#define CORRECTION_RATE     20
#define PID_RATE            400
#define INPUT_RATE          50

#define CORRECTION_DIVIDER  AHRS_RATE/CORRECTION_RATE
#define PID_DIVIDER         AHRS_RATE/PID_RATE
#define INPUT_DIVIDER       AHRS_RATE/INPUT_RATE

unsigned short correctionSoftscale, pidSoftscale, inputSoftscale;

#define THROTTLEOFFSET  800
#define OFFTHROTTLE     100
#define IDLETHROTTLE    260
#define MAXTHROTTLE     1100

#define MIDSTICK        512

//#define CONSTRAIN       600   //Constraint on maximum motor demands


// *** Parameters
typedef struct paramStorage_struct {
    char name[16];
    float value;
} paramStorage_t;

struct paramStorage_struct paramStorage[] = {
    {"DRIFT_AKp",      0.04f},     // 0
    {"DRIFT_AKi",      0.0f},     // 1
    {"DRIFT_ADe",      0.0f},     // 2
    {"DRIFT_MKp",      0.04f},     // 3
    {"DRIFT_MKi",      0.0f},     // 4
    {"DRIFT_MDe",      0.8f},     // 5
    
    {"CURVE_X1",       25.0f},     // 6
    {"CURVE_Y1",       40.0f},     // 7
    {"CURVE_X2",       50.0f},     // 8
    {"CURVE_Y2",       70.0f},     // 9
    {"CURVE_Yn",       70.0f},     // 10
    
    {"YAW_SEN",     0.00002f},     // 11
    {"PITCH_SEN",     0.003f},     // 12
    {"ROLL_SEN",      0.003f},     // 13
    {"THRUST_SEN",     0.25f},     // 14   // no longer used
    
	{"YAW_DZN",      0.0004f},
    
    {"PITCH_DZN",       0.5f},     // 16
    {"ROLL_DZN",        0.5f},     // 17
    {"THRUST_DZN",     0.05f},     // 18
    
    {"PITCH_Kp",       200.0f},     // 19
	{"PITCH_Ki",        0.5f},     // 21
	{"PITCH_Kd",        40.0f},     // 23
	{"PITCH_Bst",       100.0f},     // 27
	{"PITCH_Kdd",       0.0f},     // 25
	{"PITCH_De",     0.999f},     // 29
	
    {"ROLL_Kp",        200.0f},     // 20    
    {"ROLL_Ki",         0.5f},     // 22    
    {"ROLL_Kd",         40.0f},     // 24   
    {"ROLL_Kdd",        0.0f},     // 26		
    {"ROLL_Bst",        0.00f},     // 28		
    {"ROLL_De",      0.999f},     // 30
    
    {"YAW_Kp",         600.0f},     // 31
    {"YAW_Kd",          120.0f},     // 32
    {"YAW_Bst",         0.00f},     // 33		
    {"YAW_De",          1.00f},     // 34
    {"YAW_LIM",        300.0f},     // 35			is this made redundant by constrain?
    
	// {"DRIFT_AKp",      0.0f},     // 0
    // {"DRIFT_AKi",      0.0f},     // 1
    // {"DRIFT_ADe",      0.0f},     // 2
    // {"DRIFT_MKp",      0.0f},     // 3
    // {"DRIFT_MKi",      0.0f},     // 4
    // {"DRIFT_MDe",      0.0f},     // 5
    
    // {"CURVE_X1",       25.0f},     // 6
    // {"CURVE_Y1",       40.0f},     // 7
    // {"CURVE_X2",       50.0f},     // 8
    // {"CURVE_Y2",       70.0f},     // 9
    // {"CURVE_Yn",       70.0f},     // 10
    
    // {"YAW_SEN",     0.00002f},     // 11
    // {"PITCH_SEN",     0.003f},     // 12
    // {"ROLL_SEN",      0.003f},     // 13
    // {"THRUST_SEN",     0.25f},     // 14   // no longer used
    
	// {"YAW_DZN",      0.0004f},
    
    // {"PITCH_DZN",       0.5f},     // 16
    // {"ROLL_DZN",        0.5f},     // 17
    // {"THRUST_DZN",     0.05f},     // 18
    
    // {"PITCH_Kp",       0.0f},     // 19
	// {"PITCH_Ki",        0.5f},     // 21
	// {"PITCH_Kd",        0.0f},     // 23
	// {"PITCH_Bst",       0.0f},     // 27
	// {"PITCH_Kdd",       0.0f},     // 25
	// {"PITCH_De",     0.999f},     // 29
	
    // {"ROLL_Kp",        0.0f},     // 20    
    // {"ROLL_Ki",         0.5f},     // 22    
    // {"ROLL_Kd",         0.0f},     // 24   
    // {"ROLL_Kdd",        0.0f},     // 26		
    // {"ROLL_Bst",        0.00f},     // 28		
    // {"ROLL_De",      0.999f},     // 30
    
    // {"YAW_Kp",         0.0f},     // 31
    // {"YAW_Kd",          0.0f},     // 32
    // {"YAW_Bst",         0.00f},     // 33		
    // {"YAW_De",          1.00f},     // 34
    // {"YAW_LIM",        300.0f},     // 35			is this made redundant by constrain?
    
    {"MODE_SIMP",       0.0f},     // 36
	
	
	// float SPR_G = DRIFT_AccelKi;
	// float SPR_A = DRIFT_AccelDe;
	// float SPR_M = DRIFT_MagKi;
	// float SPR_OUT = DRIFT_MagDe;
	
	
	// {"SPR_G", 			1.0f},		//37
	// {"SPR_A", 			0.3f},		//38
	// {"SPR_M", 			0.3f},		//39
	// {"SPR_OUT", 		0.4f},		//40

	
};

unsigned int paramSendCount;
unsigned int paramCount;
unsigned char paramSendSingle;

#define DRIFT_AccelKp   paramStorage[0].value
#define DRIFT_AccelKi   paramStorage[1].value
#define DRIFT_AccelDe   paramStorage[2].value
#define DRIFT_MagKp     paramStorage[3].value
#define DRIFT_MagKi     paramStorage[4].value
#define DRIFT_MagDe     paramStorage[5].value

#define THROTTLE_X1     paramStorage[6].value
#define THROTTLE_Y1     paramStorage[7].value
#define THROTTLE_X2     paramStorage[8].value
#define THROTTLE_Y2     paramStorage[9].value
#define THROTTLE_Yn     paramStorage[10].value

#define YAW_SENS        paramStorage[11].value
#define PITCH_SENS      paramStorage[12].value
#define ROLL_SENS       paramStorage[13].value
#define THRUST_SENS     paramStorage[14].value  // no longer used

#define YAW_DEADZONE    paramStorage[15].value 
#define PITCH_DEADZONE  paramStorage[16].value 
#define ROLL_DEADZONE   paramStorage[17].value 
#define THRUST_DEADZONE paramStorage[18].value 

#define PITCH_Kp        paramStorage[19].value 
#define PITCH_Ki        paramStorage[20].value
#define PITCH_Kd        paramStorage[21].value 
#define PITCH_Kdd       paramStorage[22].value 
#define PITCH_Boost     paramStorage[23].value  
#define PITCH_De        paramStorage[24].value 

#define ROLL_Kp         paramStorage[25].value 
#define ROLL_Ki         paramStorage[26].value 
#define ROLL_Kd         paramStorage[27].value 
#define ROLL_Kdd        paramStorage[28].value 		
#define ROLL_Boost      paramStorage[29].value 		
#define ROLL_De         paramStorage[30].value 

#define YAW_Kp          paramStorage[31].value 
#define YAW_Kd          paramStorage[32].value 
#define YAW_Boost       paramStorage[33].value 		
#define YAW_De          paramStorage[34].value 
#define YAW_LIM         paramStorage[35].value

#define MODE_SIMP 		paramStorage[36].value

// #define SPR_G			paramStorage[37].value
// #define SPR_A			paramStorage[38].value
// #define SPR_M			paramStorage[39].value
// #define SPR_OUT			paramStorage[40].value



// *** Quaternion storage
typedef struct{
    float u;
    float a;
    float i;
    float r;
} quaternionStruct;

quaternionStruct quaternion;
quaternionStruct quaternionOld;

typedef struct{
    float demand;
	float demandOld;
	float gyroOld;
	float derivative;
	float integral;
} directionStruct;

directionStruct pitch;
directionStruct roll;
directionStruct yaw;
//directionStruct drift;

float thetaAngle, phiAngle, psiAngle, psiAngleinit;

// *** Sensor Storage
typedef struct{
    signed short raw;
    float av;
    float si;
    float value;
    float offset;
    float error;
    float verterror;
    float verterrorInt;
} sensorStructGyro;

typedef struct{
    sensorStructGyro X;
    sensorStructGyro Y;
    sensorStructGyro Z;
	float SPR;
} threeAxisSensorStruct;

typedef struct{
    signed short raw;
    float av;
    float value;
} sensorStruct;

threeAxisSensorStruct Gyro;
threeAxisSensorStruct Accel;
threeAxisSensorStruct Magneto;

// *** Input stuff
float nonLinearThrottle(float input);
unsigned int signalLoss;

unsigned short rcInput[7];
unsigned char auxState;


float OutputSPR;
float pitchcorrectionav, rollcorrectionav, yawcorrectionav;

unsigned char initialise;

// ****************************************************************************
// *** Initialiseation
// ****************************************************************************

void setup() {
    // *** LED setup
    LEDInit(PLED | VLED);
    LEDOff(PLED | VLED);
    
    flashPLED = 0;
    flashVLED = 0;
    flashRLED = 0;
    
    // *** Timers and couters6
    RxWatchdog = 0;
    sysMS = 0;
    sysUS = 0;
    SysTickInit();  // SysTick enable (default 1ms)

    // *** Parameters
    paramCount = sizeof(paramStorage)/20;
    paramSendCount = paramCount;
    paramSendSingle = 0;
    
    // *** Establish ILink
    LinkInit();
    ILinkInit(SLAVE);
    
    // *** Initialise input
    RXInit();
    signalLoss = 50;
    
    // *** Calibrate Sensors
    SensorInit();
    Calibrate();

    // *** Quaternion AHRS init
    quaternion.u = 1;
	quaternion.a = 0;
	quaternion.i = 0;
	quaternion.r = 0;
    
    thetaAngle = 0;
    phiAngle = 0;
    psiAngle = 0;
	
	pitchcorrectionav = 0;
	rollcorrectionav = 0;
	yawcorrectionav = 0;
	  
	// Initialise motor outputs
    //PWMInit(PWM_ALL);
    PWMInit(PWM_NESW | PWM_X);
	Delay(2000);
    
    // *** Timer for AHRS
    // Set high drift gains to rotate AHRS to "true"
    float tempAccelKp = DRIFT_AccelKp;
    float tempMagKp = DRIFT_MagKp;
 	DRIFT_MagKp *= 50;
	DRIFT_AccelKp *= 50;
	initialise = 1;
	
    Timer0Init(71);
    Timer0Match0(1000000/AHRS_RATE, INTERRUPT | RESET);
    Delay(2000);
	
    DRIFT_MagKp = tempMagKp;
    DRIFT_AccelKp = tempAccelKp;
	psiAngleinit = psiAngle;
    initialise = 0;
	
    correctionSoftscale = 0;
    pidSoftscale = 1;
    inputSoftscale = 3;
    
    RITInitms(1000/MESSAGE_LOOP_HZ);
    flashPLED = 0;
    LEDOff(PLED);
}

void LinkInit(void) {
    ilink_identify.deviceID = WHO_AM_I;
    ilink_identify.firmVersion = FIRMWARE_VERSION;
    
    ilink_thalstat.sensorStatus = 0;
    ilink_thalstat.flightMode = 0;
}

void Calibrate(void) {

	Magneto.X.av = 0;
	Magneto.Y.av = 0;
	Magneto.Z.av = 0;
	
	Accel.X.av = 0;
	Accel.Y.av = 0;
	Accel.Z.av = 0;
	
	Gyro.X.av = 0;
	Gyro.Y.av = 0;
	Gyro.Z.av = 0;
	
	
    // Calibrate Gyro
    signed int Xtotal, Ytotal, Ztotal;
    signed short data[4], i;
    
    ilink_thalstat.sensorStatus = 0x1;
    
    Xtotal = 0;
    Ytotal = 0;
    Ztotal = 0;
    
    if(GetGyro(data)) {
        LEDOn(PLED);
        LEDOff(VLED);
        flashPLED = 1;
        flashVLED = 1;
        for(i=0; i<2000; i++) {
            GetGyro(data);
            
            Xtotal += data[0];
            Ytotal += data[1];
            Ztotal += data[2];
            
            Delay(2);
        }
        Xtotal = 0;
        Ytotal = 0;
        Ztotal = 0;
        
        flashPLED = 2;
        flashVLED = 2;
        
        for(i=0; i<2000; i++) {
            GetGyro(data);
            
            Xtotal += data[0];
            Ytotal += data[1];
            Ztotal += data[2];
            
            Delay(2);
            
        }
        flashPLED = 0;
        flashVLED = 0;
        LEDOff(PLED);
        LEDOff(VLED);
    
        Gyro.X.offset = (float)Xtotal/(float)2000;
        Gyro.Y.offset = (float)Ytotal/(float)2000;
        Gyro.Z.offset = (float)Ztotal/(float)2000;
        
        ilink_thalstat.sensorStatus |= (0x1 << 1);
    }
    if(GetAccel(data)) {
        ilink_thalstat.sensorStatus |= (0x1 << 2);
    }
    if(GetMagneto(data)) {
        ilink_thalstat.sensorStatus |= (0x1 << 3);
    }
    /*if(GetBaro(data)) {
        ilink_thalstat.sensorStatus |= (0x1 << 3);
    }*/
    
    ilink_thalstat.sensorStatus &= ~0x1;
}
// ****************************************************************************
// *** Main loop and timer loops
// ****************************************************************************

void loop() {
//    if(idleCount < IDLE_MAX) idleCount++; // this is the counter for CPU idle time
}

void SysTickInterrupt(void) {
    sysMS += 1;
    sysUS += 1000;
    
    // deal with flashing LEDs

    if(sysMS % 25 == 0) {
        if(sysMS % 100 == 0) {
            if(flashPLED) LEDToggle(PLED);
            if(flashVLED) LEDToggle(VLED);
            if(flashRLED) LEDToggle(RLED);
        }
        else {
            if(flashPLED == 2) LEDToggle(PLED);
            if(flashVLED == 2) LEDToggle(VLED);
            if(flashRLED == 2) LEDToggle(RLED);
        }
    }
}

void RITInterrupt(void) {
    unsigned int i;
    
    
    
    if(paramSendCount < paramCount) {
        unsigned short thisParam = paramSendCount; // store this to avoid race hazard since paramSendCount can change outside this interrupt
        ilink_thalparam_tx.paramID = thisParam;
        ilink_thalparam_tx.paramValue = paramStorage[thisParam].value;
        ilink_thalparam_tx.paramCount = paramCount;
        for(i=0; i<16; i++) {
            ilink_thalparam_tx.paramName[i] = paramStorage[thisParam].name[i];
            if(paramStorage[thisParam].name[i] == '\0') break;
        }
        if(ILinkSendMessage(ID_ILINK_THALPARAM, (unsigned short *) & ilink_thalparam_tx, sizeof(ilink_thalparam_tx)/2 -1)) {
            if(paramSendSingle) {
                paramSendSingle = 0;
                paramSendCount = paramCount;
            }
            else {
                paramSendCount = thisParam+1;
            }
        }
    }
}


// ****************************************************************************
// *** AHRS 
// ****************************************************************************

void Timer0Interrupt0() {
    signed short sensor[4];
	float pitch_demandtemp;
    float roll_demandtemp;
    
	float SPR_G = DRIFT_AccelKi;
	float SPR_A = DRIFT_AccelDe;
	float SPR_M = DRIFT_MagKi;
	float SPR_OUT = DRIFT_MagDe;
	// float ANGLELIM = PITCH_Boost;
	
    GetGyro(sensor);
    Gyro.X.raw = sensor[0];
    Gyro.Y.raw = sensor[1];
    Gyro.Z.raw = sensor[2];
    // ilink_rawimu.xGyro = Gyro.X.raw;
    // ilink_rawimu.yGyro = Gyro.Y.raw;
    // ilink_rawimu.zGyro = Gyro.Z.raw;
    
    //Temperature.raw = sensor[3];
    
    // Gyro averaging using single pole recursive
    Gyro.X.av *= SPR_G;
    Gyro.X.av += (1-SPR_G) * Gyro.X.raw;
    Gyro.Y.av *= SPR_G;
    Gyro.Y.av += (1-SPR_G) * Gyro.Y.raw;
    Gyro.Z.av *= SPR_G;
    Gyro.Z.av += (1-SPR_G) * Gyro.Z.raw;
	
	// Gyro.X.av *= 1.0;
    // Gyro.X.av += (1-1.0) * Gyro.X.raw;
    // Gyro.Y.av *= 1.0;
    // Gyro.Y.av += (1-1.0) * Gyro.Y.raw;
    // Gyro.Z.av *= 1.0;
    // Gyro.Z.av += (1-1.0) * Gyro.Z.raw;
    
    
    
    // /*Temperature.total -= Temperature.av[Gyro.count];
    // Temperature.av[Gyro.count] = Temperature.raw;
    // Temperature.total += Temperature.raw;
    
    // if(++Gyro.count >= GYRO_AVERAGING) Gyro.count = 0;*/
    
    // //Gyro.X.av = ((float)Gyro.X.total / (float)GYRO_AVERAGING) - Gyro.X.offset;
    // //Gyro.Y.av = ((float)Gyro.Y.total / (float)GYRO_AVERAGING) - Gyro.Y.offset;
    // //Gyro.Z.av = ((float)Gyro.Z.total / (float)GYRO_AVERAGING) - Gyro.Z.offset;
    
    // //ttemp = ((float)Temperature.total / (float)GYRO_AVERAGING) - (float)TUNE_T1;
    // //Gyro.X.av = ((float)Gyro.X.total / (float)GYRO_AVERAGING) - ((float)TUNE_X1 + (float)TUNE_AX * ttemp);
    // //Gyro.Y.av = ((float)Gyro.Y.total / (float)GYRO_AVERAGING) - ((float)TUNE_Y1 + (float)TUNE_AY * ttemp);
    // //Gyro.Z.av = ((float)Gyro.Z.total / (float)GYRO_AVERAGING) - ((float)TUNE_Z1 + (float)TUNE_AZ * ttemp);
    
    Gyro.X.si = (float)(Gyro.X.av - Gyro.X.offset)/818.51113590117601252569f; // Into radians per second
    Gyro.Y.si = (float)(Gyro.Y.av - Gyro.Y.offset)/818.51113590117601252569f;
    Gyro.Z.si = (float)(Gyro.Z.av - Gyro.Z.offset)/818.51113590117601252569f;
	
	ilink_rawimu.xGyro = Gyro.X.si*800;
    ilink_rawimu.yGyro = Gyro.Y.si*800;
    ilink_rawimu.zGyro = Gyro.Z.si*800;
    
    // *** QUATERNION!
    
    Gyro.X.value = (Gyro.X.si - (float)Gyro.X.error)/(float)AHRS_RATE;
    Gyro.Y.value = (Gyro.Y.si - (float)Gyro.Y.error)/(float)AHRS_RATE;
    Gyro.Z.value = (Gyro.Z.si - (float)Gyro.Z.error)/(float)AHRS_RATE;
    
    quaternionOld.u = quaternion.u;
    quaternionOld.a = quaternion.a;
    quaternionOld.i = quaternion.i;
    quaternionOld.r = quaternion.r;

    quaternion.u -= 0.5*(Gyro.X.value*quaternionOld.a + Gyro.Y.value*quaternionOld.i + Gyro.Z.value*quaternionOld.r);
    quaternion.a += 0.5*(Gyro.X.value*quaternionOld.u + Gyro.Z.value*quaternionOld.i - Gyro.Y.value*quaternionOld.r);
    quaternion.i += 0.5*(Gyro.Y.value*quaternionOld.u - Gyro.Z.value*quaternionOld.a + Gyro.X.value*quaternionOld.r);
    quaternion.r += 0.5*(Gyro.Z.value*quaternionOld.u + Gyro.Y.value*quaternionOld.a - Gyro.X.value*quaternionOld.i);
    
    // precalculated values for optimisation
    float quu = quaternion.u * quaternion.u;
    float qua = quaternion.u * quaternion.a;
    float qui = quaternion.u * quaternion.i;
    // float qur = quaternion.u * quaternion.r; // No gain from precalc
    float qaa = quaternion.a * quaternion.a;
    // float qai = quaternion.a * quaternion.i; // No gain from precalc
    float qar = quaternion.a * quaternion.r;
    float qii = quaternion.i * quaternion.i;
    float qir = quaternion.i * quaternion.r;
    float qrr = quaternion.r * quaternion.r;
    float qaaPqii = qaa + qii;
    float TquiMqar = 2 * (qui - qar);
    float TqirPqua = 2 * (qir + qua);
    
    // renormalise using Taylor expansion
    // sumsqu = (3-(quu + qaaPqii + qrr))/2;
    // quaternion.u *= sumsqu;
    // quaternion.a *= sumsqu;
    // quaternion.i *= sumsqu;
    // quaternion.r *= sumsqu;
    
    // renormalise using fast inverse square root
    float sumsqu = invSqrt(quu + qaaPqii + qrr);
    quaternion.u *= sumsqu;
    quaternion.a *= sumsqu;
    quaternion.i *= sumsqu;
    quaternion.r *= sumsqu;

    // avoid gimbal lock at singularity points
    if (TquiMqar == 1) {
        psiAngle = 2 * fatan2(quaternion.a, quaternion.u);
        thetaAngle = M_PI_2;
        phiAngle = 0;
    }
    else if (TquiMqar == -1) {
        psiAngle = -2 * fatan2(quaternion.a, quaternion.u);
        thetaAngle = - M_PI_2;
        phiAngle = 0;
    }
    else {
        thetaAngle = fasin(TquiMqar);    
        phiAngle = fatan2(TqirPqua, (1 - 2*qaaPqii));  
        psiAngle = fatan2((2*(quaternion.u * quaternion.r + quaternion.a * quaternion.i)), (1 - 2*(qii + qrr)));  
    }
    ilink_attitude.roll = phiAngle;
    ilink_attitude.pitch = thetaAngle;
    ilink_attitude.yaw = psiAngle;

    if(++correctionSoftscale >= CORRECTION_DIVIDER) {
        correctionSoftscale = 0;
        // *** Read Accelerometers
        
        GetAccel(sensor);
        Accel.X.raw = sensor[0];
        Accel.Y.raw = sensor[1];
        Accel.Z.raw = sensor[2];
        ilink_rawimu.xAcc = Accel.X.raw;
        ilink_rawimu.yAcc = Accel.Y.raw;
        ilink_rawimu.zAcc = Accel.Z.raw;

        Accel.X.av *= SPR_A;
        Accel.X.av += (1-SPR_A) * Accel.X.raw;
        Accel.Y.av *= SPR_A;
        Accel.Y.av += (1-SPR_A) * Accel.Y.raw;
        Accel.Z.av *= SPR_A;
        Accel.Z.av += (1-SPR_A) * Accel.Z.raw;
        
        sumsqu = invSqrt((float)Accel.X.av*(float)Accel.X.av + (float)Accel.Y.av*(float)Accel.Y.av + (float)Accel.Z.av*(float)Accel.Z.av); // Accelerometr data is normalised so no need to convert units.
        Accel.X.value = (float)Accel.X.av * sumsqu;
        Accel.Y.value = (float)Accel.Y.av * sumsqu;
        Accel.Z.value = (float)Accel.Z.av * sumsqu;
        
        
        // *** Read Magnetometers
        GetMagneto(sensor);
        Magneto.X.raw = sensor[0];
        Magneto.Y.raw = sensor[1];
        Magneto.Z.raw = sensor[2];
        ilink_rawimu.xMag = Magneto.X.raw;
        ilink_rawimu.yMag = Magneto.Y.raw;
        ilink_rawimu.zMag = Magneto.Z.raw;
        
        Magneto.X.av *= SPR_M;
        Magneto.X.av += (1-SPR_M) * Magneto.X.raw;
        Magneto.Y.av *= SPR_M;
        Magneto.Y.av += (1-SPR_M) * Magneto.Y.raw;
        Magneto.Z.av *= SPR_M;
        Magneto.Z.av += (1-SPR_M) * Magneto.Z.raw;
        
        Magneto.X.value = (float)Magneto.X.av;
        Magneto.Y.value = (float)Magneto.Y.av;
        Magneto.Z.value = (float)Magneto.Z.av;
        
        // *** Drift correction
        float m9 = (quu - qaa - qii + qrr);
        Gyro.X.verterror = TqirPqua*Accel.Z.value - m9*Accel.Y.value;  // Correction vector
        Gyro.Y.verterror = m9*Accel.X.value + TquiMqar*Accel.Z.value;
        
        
        float MdotA = Magneto.X.value*Accel.X.value + Magneto.Y.value*Accel.Y.value + Magneto.Z.value*Accel.Z.value;
        float X = (Magneto.X.value - MdotA*Accel.X.value);  // Lagranges Theorum 
        float Y = (Magneto.Y.value - MdotA*Accel.Y.value);

        //if(Accel.Z.value > 0.3) {
            Gyro.Z.verterror = psiAngle + fatan2(Y, X); // Heading correction error calculation
            //if (Accel.Z.value < 0) Gyro.Z.verterror = psiAngle - fatan2(Y, X) + M_PI; // Todo: fix upside down case

            if (Gyro.Z.verterror > M_PI) Gyro.Z.verterror -= M_TWOPI;
            if (Gyro.Z.verterror < -M_PI) Gyro.Z.verterror += M_TWOPI;
        //}
        //else {
        //    Gyro.Z.verterror = 0;
        //}

        Gyro.X.verterrorInt *= DRIFT_AccelDe;    // Integral decay
        Gyro.Y.verterrorInt *= DRIFT_AccelDe;
        Gyro.Z.verterrorInt *= DRIFT_MagDe;
        Gyro.X.verterrorInt += Gyro.X.verterror;
        Gyro.Y.verterrorInt += Gyro.Y.verterror;
        Gyro.Z.verterrorInt += Gyro.Z.verterror;
        Gyro.X.error = -DRIFT_AccelKp*Gyro.X.verterror;// - DRIFT_AccelKi*Gyro.X.verterrorInt;  // Error is inserted into a PI loop
        Gyro.Y.error = -DRIFT_AccelKp*Gyro.Y.verterror;// - DRIFT_AccelKi*Gyro.Y.verterrorInt;
        Gyro.Z.error = DRIFT_MagKp*Gyro.Z.verterror; // + DRIFT_MagKi*Gyro.Z.verterrorInt;
    }
    
	if(initialise == 0) {
		if(++pidSoftscale >= PID_DIVIDER) {
			pidSoftscale = 0;
			
			signed short motorN, motorE, motorS, motorW;
			float pitcherror, rollerror, yawerror;
			float pitchcorrection, rollcorrection, yawcorrection;
			
			// *** Flight Control
			// Control Inputs
			
			pitch.demand = -((float)MIDSTICK - (float)rcInput[RX_ELEV])*PITCH_SENS; 
			roll.demand = ((float)MIDSTICK - (float)rcInput[RX_AILE])*ROLL_SENS;
			
			
			
			// Yaw
			float tempf = -((float)MIDSTICK - (float)rcInput[RX_RUDD])*YAW_SENS; //3125
			if(fabsf(tempf) > YAW_DEADZONE) {
				yaw.demand += tempf;
				if(yaw.demand > M_PI) {
					yaw.demand -= M_TWOPI;
					yaw.demandOld -= M_TWOPI;
				}
				else if(yaw.demand < -M_PI) {
					yaw.demand += M_TWOPI;
					yaw.demandOld += M_TWOPI;
				}
			}
			
			pitch_demandtemp = pitch.demand;
			roll_demandtemp = roll.demand;			

			if (MODE_SIMP == 1) {
				ilink_thalstat.flightMode = (0x1 << 0) | (0x1 << 2); // attitude and yaw control
				pitch.demand = fsin(-psiAngle+psiAngleinit+M_PI_2)*pitch_demandtemp - fsin(-psiAngle+psiAngleinit)*roll_demandtemp;
				roll.demand = fsin(-psiAngle+psiAngleinit)*pitch_demandtemp + fsin(-psiAngle+psiAngleinit+M_PI_2)*roll_demandtemp;
			}
			else {
				ilink_thalstat.flightMode = (0x1 << 0); // attitude control
				pitch.demand = fsin(0.78539816339744830962+M_PI_2)*pitch_demandtemp - fsin(0.78539816339744830962)*roll_demandtemp;
				roll.demand = fsin(0.78539816339744830962)*pitch_demandtemp + fsin(0.78539816339744830962+M_PI_2)*roll_demandtemp;
			}
			// for acro mode, ilink_thalstat.flightMode = (0x1 << 1); // angle rate control
			// for altitude hold, ilink_thalstat.flightMode = (0x1 << 3); // altitude hold
			
			float throttle = nonLinearThrottle((float)rcInput[RX_THRO]);
			//float throttle = (float)rcInput[RX_THRO];
			//float throttle = 500;
			
			// if(pitch.demand > ANGLELIM) pitch.demand = ANGLELIM;		//limit roll and pitch angles
			// if(pitch.demand < -ANGLELIM) pitch.demand = -ANGLELIM;
			// if(roll.demand > ANGLELIM) roll.demand = ANGLELIM;
			// if(roll.demand < -ANGLELIM) roll.demand = -ANGLELIM;
	 
			pitch.derivative = (pitch.demand - pitch.demandOld);    // times 50 for 50 Hz
			roll.derivative = (roll.demand - roll.demandOld);
			yaw.derivative = (yaw.demand - yaw.demandOld);
			
			pitch.demandOld = pitch.demand;
			roll.demandOld = roll.demand;
			yaw.demandOld = yaw.demand;
		
			// Errors for Motor PID loops
			pitcherror = pitch.demand + thetaAngle;
			rollerror = roll.demand - phiAngle;
			yawerror = yaw.demand + psiAngle; 

			if(pitcherror > M_PI) pitcherror -= M_TWOPI;
			else if(pitcherror < -M_PI) pitcherror += M_TWOPI;
			
			if(rollerror > M_PI) rollerror -= M_TWOPI;
			else if(rollerror < -M_PI) rollerror += M_TWOPI;
			
			if(yawerror > M_PI) yawerror -= M_TWOPI;
			else if(yawerror < -M_PI) yawerror += M_TWOPI;

			pitch.integral *= PITCH_De;
			roll.integral *= ROLL_De;
			yaw.integral *= YAW_De;
			pitch.integral += pitcherror;
			roll.integral += rollerror;
			yaw.integral += yawerror;
			
			// Attitude control PID loops
			// pitchcorrection = -((float)Gyro.Y.si - PITCH_Boost*pitch.derivative) * PITCH_Kd;
			pitchcorrection = -((float)Gyro.Y.si) * PITCH_Kd;
			pitchcorrection += -PITCH_Kdd*((float)Gyro.Y.si - pitch.gyroOld);
			pitchcorrection += -PITCH_Kp*pitcherror;
			pitchcorrection += -PITCH_Ki*pitch.integral;

			// rollcorrection = -((float)Gyro.X.si - ROLL_Boost*roll.derivative) * ROLL_Kd;
			rollcorrection = -((float)Gyro.X.si) * ROLL_Kd;
			rollcorrection += -ROLL_Kdd*((float)Gyro.X.si - roll.gyroOld);
			rollcorrection += ROLL_Kp*rollerror;
			rollcorrection += ROLL_Ki*roll.integral;

			// yawcorrection = -((float)Gyro.Z.si + YAW_Boost*yaw.derivative) * YAW_Kd;
			yawcorrection = -((float)Gyro.Z.si) * YAW_Kd;
			//yawcorrection += YAW_Kdd*((float)Gyro.Z.av - yaw.gyroOld);
			yawcorrection += -YAW_Kp*yawerror;
			//yawcorrection += YAW_Ki*yaw.integral;
			if(yawcorrection > YAW_LIM) yawcorrection = YAW_LIM;
			else if(yawcorrection < -YAW_LIM) yawcorrection = -YAW_LIM;
			
			pitch.gyroOld = (float)Gyro.Y.si;
			roll.gyroOld = (float)Gyro.X.si;
			yaw.gyroOld = (float)Gyro.Z.si;
			
			pitchcorrectionav *= SPR_OUT;
			pitchcorrectionav += (1-SPR_OUT) * pitchcorrection;
			rollcorrectionav *= SPR_OUT;
			rollcorrectionav += (1-SPR_OUT) * rollcorrection;
			yawcorrectionav *= SPR_OUT;
			yawcorrectionav += (1-SPR_OUT) * yawcorrection;
			
			
			//pitchcorrection = 0;
			//yawcorrection = 0;
			//rollcorrection = 0;
			
			// motorN = pitchcorrection - yawcorrection; 
			// motorE = -rollcorrection + yawcorrection;
			// motorS = -pitchcorrection - yawcorrection;
			// motorW = rollcorrection + yawcorrection;
			
			motorN = pitchcorrectionav - yawcorrectionav; 
			motorE = -rollcorrectionav + yawcorrectionav;
			motorS = -pitchcorrectionav - yawcorrectionav;
			motorW = rollcorrectionav + yawcorrectionav;
			
			// motorN = 0;
			// motorE = 0;
			// motorS = 0;
			// motorW = 0;
			
			
				// if(motorN > CONSTRAIN) motorN = CONSTRAIN;
				// if(motorN < -CONSTRAIN) motorN = -CONSTRAIN;
				// if(motorE > CONSTRAIN) motorE = CONSTRAIN;
				// if(motorE < -CONSTRAIN) motorE = -CONSTRAIN;
				// if(motorS > CONSTRAIN) motorS = CONSTRAIN;
				// if(motorS < -CONSTRAIN) motorS = -CONSTRAIN;
				// if(motorW > CONSTRAIN) motorW = CONSTRAIN;
				// if(motorW < -CONSTRAIN) motorW = -CONSTRAIN;
			
			
			if (throttle  <  OFFTHROTTLE  || signalLoss >= 50){
			
				pitch.integral=0;
				roll.integral=0;
				yaw.integral=0;
				Gyro.X.verterrorInt = 0;
				Gyro.Y.verterrorInt = 0;
				Gyro.Z.verterrorInt = 0;
				
				if(throttle < 0) {
					throttle = 0;
				}
				yaw.demand = -psiAngle;
				yaw.demandOld = -psiAngle;
				PWMSetN(OFFTHROTTLE + THROTTLEOFFSET);
				PWMSetE(OFFTHROTTLE + THROTTLEOFFSET);
				PWMSetS(OFFTHROTTLE + THROTTLEOFFSET);
				PWMSetX(OFFTHROTTLE + THROTTLEOFFSET); // temporary output for bad connection on S
				PWMSetW(OFFTHROTTLE + THROTTLEOFFSET);
				ilink_outputs0.channel[0] = OFFTHROTTLE + THROTTLEOFFSET;
				ilink_outputs0.channel[1] = OFFTHROTTLE + THROTTLEOFFSET;
				ilink_outputs0.channel[2] = OFFTHROTTLE + THROTTLEOFFSET;
				ilink_outputs0.channel[3] = OFFTHROTTLE + THROTTLEOFFSET;
			}
			else {
				float temp;
				temp = (signed short)motorN + (signed short)throttle + THROTTLEOFFSET;
				if(temp > (MAXTHROTTLE + THROTTLEOFFSET)) temp = (MAXTHROTTLE + THROTTLEOFFSET);
				else if(temp < (IDLETHROTTLE + THROTTLEOFFSET)) temp = (IDLETHROTTLE + THROTTLEOFFSET);
				PWMSetN(temp);
				ilink_outputs0.channel[0] = temp;
				
				temp = (signed short)motorE + (signed short)throttle +  THROTTLEOFFSET;
				if(temp > (MAXTHROTTLE + THROTTLEOFFSET)) temp = (MAXTHROTTLE + THROTTLEOFFSET);
				else if(temp < (IDLETHROTTLE + THROTTLEOFFSET)) temp = (IDLETHROTTLE + THROTTLEOFFSET);
				PWMSetE(temp);
				ilink_outputs0.channel[1] = temp;
				
				temp = (signed short)motorS + (signed short)throttle + THROTTLEOFFSET;
				if(temp > (MAXTHROTTLE + THROTTLEOFFSET)) temp = (MAXTHROTTLE + THROTTLEOFFSET);
				else if(temp < (IDLETHROTTLE + THROTTLEOFFSET)) temp = (IDLETHROTTLE + THROTTLEOFFSET);
				PWMSetS(temp);
				PWMSetX(temp); // temporary output for bad connection on S
				ilink_outputs0.channel[2] = temp;
				
				temp = (signed short)motorW + (signed short)throttle + THROTTLEOFFSET;
				if(temp > (MAXTHROTTLE + THROTTLEOFFSET)) temp = (MAXTHROTTLE + THROTTLEOFFSET);
				else if(temp < (IDLETHROTTLE + THROTTLEOFFSET)) temp = (IDLETHROTTLE + THROTTLEOFFSET);
				PWMSetW(temp);
				ilink_outputs0.channel[3] = temp;
			}
			
			//PWMSetX(MINTHROTTLE + throttle);
		   
			
			// // Gimbal control
			// float pitch_temp = fsin(0.78539+M_PI_2)*thetaAngle - fsin(0.78539)*phiAngle;
			// float roll_temp = fsin(0.78539)*thetaAngle + fsin(0.78539+M_PI_2)*phiAngle;
			// //float pitch_temp = thetaAngle;
			// //float roll_temp = phiAngle;
			
			// if(auxState == 0) {
				// float value = (float)1200 + (float)roll_temp * (float)1900 / M_PI;
			
				// //if(auxState == 1) {value += 900;}
				// if(value > 2300) value = 2300;
				// if(value < 700) value = 700;

				// PWMSetX(value);
				// ilink_outputs0.channel[4] = value;
					
				// value = (float)1350 - (float)pitch_temp * (float)1900 / M_PI;
				// if(value > 2300) value = 2300;
				// if(value < 1050) value = 1050;
				// PWMSetY(value);
				// ilink_outputs0.channel[5] = value;
				
			// }
			// else {
				// PWMSetY(1350);
				// ilink_outputs0.channel[4] = 1250;
				// PWMSetX(1200);
				// ilink_outputs0.channel[5] = 1200;
			// }
			
			ilink_outputs0.isNew = 1;
		}
		
		if(++inputSoftscale >= INPUT_DIVIDER) {
			inputSoftscale = 0;
			if(RXGetData(rcInput)) {
				if(signalLoss > 10) signalLoss -= 10;
				
				ilink_inputs0.channel[0] = rcInput[RX_THRO];
				ilink_inputs0.channel[1] = rcInput[RX_AILE];
				ilink_inputs0.channel[2] = rcInput[RX_ELEV];
				ilink_inputs0.channel[3] = rcInput[RX_RUDD];
				ilink_inputs0.channel[4] = rcInput[RX_GEAR];
				ilink_inputs0.channel[5] = rcInput[RX_FLAP];
				ilink_inputs0.isNew = 1;
				
				// Controller's aux or gear switch
				if(rcInput[RX_AUX1] > MIDSTICK) {
					if(auxState == 1) {
						// do something on switch
					}
					auxState = 0;
				}
				else {
					if(auxState == 0) {
						// do something on switch
					}
					auxState = 1;
				}
				flashVLED = 0;
				LEDOff(VLED);
			}
			else {
				signalLoss ++;
				if(signalLoss > 50) {
					signalLoss = 50;
					// RC signal lost
					PWMSetNESW(THROTTLEOFFSET+OFFTHROTTLE, THROTTLEOFFSET+OFFTHROTTLE, THROTTLEOFFSET+OFFTHROTTLE, THROTTLEOFFSET+OFFTHROTTLE);
					PWMSetX(THROTTLEOFFSET+OFFTHROTTLE); // temporary output for bad connection on S
					flashVLED = 2;
				}
			}
		}
	}
}

// ****************************************************************************
// *** Other Functions
// ****************************************************************************

float nonLinearThrottle(float input) {
    float output;
	
    if (input < 10*THROTTLE_X1) {
		output = input * THROTTLE_Y1;
        output /= THROTTLE_X1;
	}
    else if(input < 10*THROTTLE_X2) {
		output = (input-(THROTTLE_X1*10));
		output *= (THROTTLE_Y2-THROTTLE_Y1);
		output /= (THROTTLE_X2-THROTTLE_X1);
		output += (10*THROTTLE_Y1);
	}
    else {
        output = (THROTTLE_Yn-THROTTLE_Y2);
		output *= (input-(10*THROTTLE_X2));
		output /= (100-THROTTLE_X2);
		output += (10*THROTTLE_Y2);
	}
    return output;
}


// ****************************************************************************
// *** Communications
// ****************************************************************************

void ILinkMessageRequest(unsigned short id) {
    unsigned short * ptr = 0;
    unsigned short maxlength = 0;
    
    switch(id) {
        case ID_ILINK_IDENTIFY:
            ptr = (unsigned short *) &ilink_identify;
            maxlength = sizeof(ilink_identify)/2 - 1;
            LEDToggle(PLED);
            break;
        case ID_ILINK_THALSTAT:
            ptr = (unsigned short *) &ilink_thalstat;
            maxlength = sizeof(ilink_thalstat)/2 - 1;
            break;
        case ID_ILINK_RAWIMU:
            ptr = (unsigned short *) &ilink_rawimu;
            maxlength = sizeof(ilink_rawimu)/2 - 1;
            break;
        case ID_ILINK_ATTITUDE:
            ptr = (unsigned short *) &ilink_attitude;
            maxlength = sizeof(ilink_attitude)/2 - 1;
            break;
        case ID_ILINK_INPUTS0:
            if(ilink_inputs0.isNew) {
                ilink_inputs0.isNew = 0;
                ptr = (unsigned short *) &ilink_inputs0;
                maxlength = sizeof(ilink_inputs0)/2 - 1;
            }
            break;
        case ID_ILINK_OUTPUTS0:
            if(ilink_outputs0.isNew) {
                ilink_outputs0.isNew = 0;
                ptr = (unsigned short *) &ilink_outputs0;
                maxlength = sizeof(ilink_outputs0)/2 - 1;
            }
            break;
        case ID_ILINK_CLEARBUF:
            FUNCILinkTxBufferPushPtr = 0;
            FUNCILinkTxBufferPopPtr = 0;
            break;
    }

    if(ptr) {
        ILinkSendMessage(id, ptr, maxlength);
    }
}

void ILinkMessage(unsigned short id, unsigned short * buffer, unsigned short length) {
    unsigned short * ptr = 0;
    unsigned int i, j;
    
    switch(id) {
        case ID_ILINK_THALPAREQ: ptr = (unsigned short *) &ilink_thalpareq; break;
        case ID_ILINK_THALPARAM: ptr = (unsigned short *) &ilink_thalparam_rx; break;
    }
    
    if(ptr) {
        for(i=0; i<length; i++) {
            ptr[i] = buffer[i];
        }
        ptr[i] = 1; // this should be the isNew byte
    }
    
    switch(id) {
        case ID_ILINK_THALPAREQ:
            if(ilink_thalpareq.reqType == 0) { 
                paramSendCount = 0;
                paramSendSingle = 0;
            }
            else {
                if(ilink_thalpareq.paramID == 0xffff) {
                
                    for (i=0; i<paramCount; i++){
                        unsigned char match = 1;
                        for (j=0; j<16; j++) {
                            if (paramStorage[i].name[j] !=  ilink_thalparam_rx.paramName[j]) {
                                match = 0;
                                break;
                            }
                            if (paramStorage[i].name[j] == '\0') break;
                        }
                        
                        if(match == 1) {
                            // when a match is found get the iD
                            paramSendCount = i;
                            paramSendSingle = 1;
                            break;
                        }
                    }
                }
                else {
                    paramSendCount = ilink_thalpareq.paramID;
                    paramSendSingle = 1;
                }
            }
            break;
        case ID_ILINK_THALPARAM:
            // match up received parameter with stored parameter.  MAVLink y u no give index?
            for (i=0; i<paramCount; i++){
                unsigned char match = 1;
                for (j=0; j<16; j++) {
                    if (paramStorage[i].name[j] !=  ilink_thalparam_rx.paramName[j]) {
                        match = 0;
                        break;
                    }
                    if (paramStorage[i].name[j] == '\0') break;
                }
                
                if(match == 1) {
                    // when a match is found, save it to paramStorage
                    paramStorage[i].value = ilink_thalparam_rx.paramValue;
                    
                    // then order the value to be sent out again using the param send engine
                    // but deal with cases where it's already in the process of sending out data
                    if(paramSendCount < paramCount) {
                        // parameter engine currently sending out data
                        if(paramSendCount >= i) {
                            // if parameter engine already sent out this now-changed data, redo this one, otherwise no action needed
                            paramSendCount = i;
                        }
                    }
                    else {
                        // parameter engine not currently sending out data, so send single parameter
                        paramSendCount = i;
                        paramSendSingle = 1;
                    }
                    break;
                }
            }
            break;
    }
}
