#include "board.h"
#include "mw.h"

float        accSmooth[3], ACC_speed[2];
float        accADC[3], gyroADC[3], magADCfloat[3];
int16_t      sonarAlt;
float        BaroAlt, EstAlt, AltHold, vario;            // variometer in cm/s + is up
int16_t      BaroP, BaroI, BaroD;
bool         newbaroalt, GroundAltInitialized;
float        BaroDeltaTime, ACCDeltaTimeINS = 0;
static float INV_GYR_CMPF_FACTOR, INV_GYR_CMPFM_FACTOR, INV_ACC_INS_LPF, INV_ACC_LPF;

// **************
// gyro+acc IMU
// **************
/*
 * Sensor data rate:
 * Baro  - 25 Hz  - 40 ms  | 50 ms
 * Accel - 400 Hz - 2.5 ms | 10 ms
 * Mag   - 30 Hz  - 4.5 ms | 40 ms
 * Gyro  - 760 Hz - 1.3 ms | 10 ms
 */

float   gyroData[3] = { 0, 0, 0 }, angle[2] = { 0, 0 };                    // absolute angle inclination in multiple of 0.1 degree    180 deg = 1800
static  uint8_t Smoothing[3]  = { 0, 0, 0 };
static  bool    GyroSmoothing;

static void getEstimatedAttitude(void);

void imuInit(void)                                                         // Initialize & precalculate some values here
{
    INV_GYR_CMPF_FACTOR  = 1.0f / (float)(cfg.gy_cmpf  + 1);               // Default 400
    INV_GYR_CMPFM_FACTOR = 1.0f / (float)(cfg.gy_cmpfm + 1);               // Default 200
    INV_ACC_INS_LPF      = 1.0f / (float)cfg.acc_ilpf;                     // acc_ilpf is limited to 1 in cli to avoid 0
    INV_ACC_LPF          = 1.0f / (float)cfg.acc_lpf;                      // acc_lpf is limited to 1 in cli to avoid 0
    accADC[0] = accADC[1] = accADC[2] = 0;

    if (cfg.gy_smrll || cfg.gy_smptc || cfg.gy_smyw)
    {
        Smoothing[ROLL]  = cfg.gy_smrll;
        Smoothing[PITCH] = cfg.gy_smptc;
        Smoothing[YAW]   = cfg.gy_smyw;
        GyroSmoothing    = true;
    } else GyroSmoothing = false;

#ifdef MAG
    if (sensors(SENSOR_MAG)) Mag_init();
#endif
}

void computeIMU(void)
{
    static  float gyroSmooth[3] = { 0, 0, 0 };
    uint8_t axis;

    if (MpuSpecial)
    {
        GETMPU6050();
        getEstimatedAttitude();
    }
    else
    {
        gyro.temperature(&telemTemperature1);                    // Read out gyro temperature
        Gyro_getADC();                                           // Also feeds gyroData
        if (sensors(SENSOR_ACC))
        {
            ACC_getADC();
            getEstimatedAttitude();
        }
    }

    if (GyroSmoothing)
    {
        for (axis = 0; axis < 3; axis++)
        {
            if (Smoothing[axis])                                 // Circumvent the 0 here
            {
                gyroData[axis]   = ((gyroSmooth[axis] * (float)(Smoothing[axis] - 1)) + gyroData[axis]) / (float)Smoothing[axis];
                gyroSmooth[axis] = gyroData[axis];
            }
        }
    }
}

typedef struct fp_vector
{
    float X, Y, Z;
} t_fp_vector_def;

typedef union
{
    float A[3];
    t_fp_vector_def V;
} t_fp_vector;

t_fp_vector EstG;

// Rotate Estimated vector(s) with small angle approximation, according to the gyro data
void rotateV(struct fp_vector *v, float *delta)
{
    struct    fp_vector v_tmp = *v;
    float     mat[3][3];                                                  // This does a  "proper" matrix rotation using gyro deltas without small-angle approximation
    float     cosx, sinx, cosy, siny, cosz, sinz;
    float     coszcosx, coszcosy, sinzcosx, coszsinx, sinzsinx;
    cosx      = cosf(-delta[PITCH]);
    sinx      = sinf(-delta[PITCH]);
    cosy      = cosf(delta[ROLL]);
    siny      = sinf(delta[ROLL]);
    cosz      = cosf(delta[YAW]);
    sinz      = sinf(delta[YAW]);
    coszcosx  = cosz * cosx;
    coszcosy  = cosz * cosy;
    sinzcosx  = sinz * cosx;
    coszsinx  = sinx * cosz;
    sinzsinx  = sinx * sinz;
    mat[0][0] = coszcosy;
    mat[0][1] = sinz * cosy;
    mat[0][2] = -siny;
    mat[1][0] = (coszsinx * siny) - sinzcosx;
    mat[1][1] = (sinzsinx * siny) + (coszcosx);
    mat[1][2] = cosy * sinx;
    mat[2][0] = (coszcosx * siny) + (sinzsinx);
    mat[2][1] = (sinzcosx * siny) - (coszsinx);
    mat[2][2] = cosy * cosx;
    v->X      = v_tmp.X * mat[0][0] + v_tmp.Y * mat[1][0] + v_tmp.Z * mat[2][0];
    v->Y      = v_tmp.X * mat[0][1] + v_tmp.Y * mat[1][1] + v_tmp.Z * mat[2][1];
    v->Z      = v_tmp.X * mat[0][2] + v_tmp.Y * mat[1][2] + v_tmp.Z * mat[2][2];
}

static void getEstimatedAttitude(void)
{
    static t_fp_vector EstM;
    static float    accLPFINS[3];
    static uint32_t previousT;
    float           scale, deltaGyroAngle[3];
    float           rollRAD, pitchRAD, cr, sr, cp, sp, Xh, Yh;
    float           cy, sy, spcy, spsy, acc_south, acc_west, acc_up;
    float           tmp0, tmp1, tmp2, tmp3, AccMag = 0;
    uint8_t         axis;
    uint32_t        currentT = micros();
  
    tmp0            = (float)(currentT - previousT);
    scale           = tmp0 * GyroScale;
    ACCDeltaTimeINS = tmp0 * 0.000001f;
    previousT       = currentT;

    tmp1 = 1.0f - INV_ACC_INS_LPF;
    tmp3 = 1.0f - INV_ACC_LPF;
    for (axis = 0; axis < 3; axis++)
    {
        deltaGyroAngle[axis] = gyroADC[axis]   * scale;
        accLPFINS[axis]      = accLPFINS[axis] * tmp1 + accADC[axis] * INV_ACC_INS_LPF;
        accSmooth[axis]      = accSmooth[axis] * tmp3 + accADC[axis] * INV_ACC_LPF;
        AccMag              += accSmooth[axis] * accSmooth[axis];
    }
    AccMag = (AccMag * 100) / SQacc_1G;
    rotateV(&EstG.V, deltaGyroAngle);
    if (sensors(SENSOR_MAG)) rotateV(&EstM.V, deltaGyroAngle);
//    if (abs(accSmooth[ROLL])  < acc_25deg &&
//        abs(accSmooth[PITCH]) < acc_25deg && accSmooth[YAW] > 0) f.SMALL_ANGLES_25 = 1;
//    else f.SMALL_ANGLES_25 = 0;
//    debug[0] = f.SMALL_ANGLES_25;    
    
    // Apply complimentary filter (Gyro drift correction)
    // If accel magnitude >1.15G or <0.85G and ACC vector outside of the limit range => we neutralize the effect of accelerometers in the angle estimation.
    // To do that, we just skip filter, as EstV already rotated by Gyro
    if (72 < AccMag && AccMag < 133)
    {
        for (axis = 0; axis < 3; axis++)
            EstG.A[axis] = (EstG.A[axis] * (float)cfg.gy_cmpf + accSmooth[axis]) * INV_GYR_CMPF_FACTOR;
    }

    if (EstG.A[YAW] > ACCZ_25deg) f.SMALL_ANGLES_25 = 1;
    else f.SMALL_ANGLES_25 = 0;

#ifdef MAG
    if (sensors(SENSOR_MAG))
    {
        for (axis = 0; axis < 3; axis++)
            EstM.A[axis] = (EstM.A[axis] * (float)cfg.gy_cmpfm + magADCfloat[axis]) * INV_GYR_CMPFM_FACTOR; // EstM.A[axis] = (EstM.A[axis] * GYR_CMPFM_FACTOR + magADCfloat[axis]) * INV_GYR_CMPFM_FACTOR;
    }
#endif
//  rollRAD      = atan2f(EstG.V.X, EstG.V.Z);
//  pitchRAD     = asinf(EstG.V.Y / -sqrtf(EstG.V.X * EstG.V.X + EstG.V.Y * EstG.V.Y + EstG.V.Z * EstG.V.Z)); // Has to have the "wrong sign" relative to angle[PITCH]
    tmp0         = EstG.V.X * EstG.V.X + EstG.V.Z * EstG.V.Z;
    rollRAD      = atan2f(EstG.V.X, EstG.V.Z);
    pitchRAD     = -atan2f(EstG.V.Y, tmp0 / sqrtf(tmp0));              // "Pitchrad" has to have the "wrong sign" relative to angle[PITCH]
    angle[ROLL]  = constrain( rollRAD  * RADtoDEG10, -1800, 1800);
    angle[PITCH] = constrain(-pitchRAD * RADtoDEG10, -1800, 1800);

    cr        = cosf(rollRAD);
    sr        = sinf(rollRAD);
    cp        = cosf(pitchRAD);
    sp        = sinf(pitchRAD);
    TiltValue = EstG.V.Z * INVacc_1G;                                  // / acc_1G;
    heading   = 0;                                                     // if no mag or not calibrated do bodyframe below
#ifdef MAG
    if (sensors(SENSOR_MAG) && cfg.mag_calibrated == 1)
    {
        Xh   = EstM.A[1] * cp + EstM.A[0] * sr * sp + EstM.A[2] * cr * sp;
        Yh   = EstM.A[0] * cr - EstM.A[2] * sr;
        heading = constrain(atan2f(-Yh,Xh) * RADtoDEG, -180.0f, 180.0f) + magneticDeclination; // Get rad to Degree and add declination (without *10 shit)
        if (heading > 180.0f)       heading = heading - 360.0f;        // Wrap to -180 0 +180 Degree
        else if (heading < -180.0f) heading = heading + 360.0f;
    }
#endif
    tmp0      = heading * RADX;                                        // Do GPS INS rotate ACC X/Y to earthframe no centrifugal comp. yet
    cy        = cosf(tmp0);
    sy        = sinf(tmp0);
    cos_yaw_x = cy;                                                    // Store for general use
    sin_yaw_y = sy;                                                    // Store for general use
    spcy      = sp * cy;
    spsy      = sp * sy;
    tmp0      = accLPFINS[0] * INVacc_1G;                              // / acc_1G Reference to Gravity before rotation
    tmp1      = accLPFINS[1] * INVacc_1G;
    tmp2      = accLPFINS[2] * INVacc_1G;
    acc_up    = ((-sp) * tmp1 + (sr * cp) * tmp0 + cp * cr * tmp2) - 1;// -1G That works good for althold
    tmp0      = accLPFINS[0];
    tmp1      = accLPFINS[1];
    tmp2      = accLPFINS[2];
    tmp3      = sqrtf(EstG.V.X * tmp0 + EstG.V.Y  * tmp1 + EstG.V.Z * tmp2); // Normalize ACCvector so the gps ins works
    if (tmp3 == 0.0f) tmp3 = 1;                                        // In that case all tmp must be zero so div by 1 is ok
    tmp0      = tmp0 / tmp3;
    tmp1      = tmp1 / tmp3;
    tmp2      = tmp2 / tmp3;
    acc_south = (cp * cy) * tmp1 + (sr * spcy - cr * sy) * tmp0 + ( sr * sy + cr * spcy) * tmp2;
    acc_west  = (cp * sy) * tmp1 + (cr * cy + sr * spsy) * tmp0 + (-sr * cy + cr * spsy) * tmp2;
    tmp3      = 980.665f  * ACCDeltaTimeINS;                           // vel factor for normalized output tmp3      = (9.80665f * (float)ACCDeltaTime) / 10000.0f;
    tmp2      = constrain(TiltValue, 0.5f, 1.0f) * tmp3;               // Empirical reduction of hightdrop in forward flight
    if(GroundAltInitialized) vario = vario + acc_up * tmp2;            // Positive when moving Up. Just do Vario when Baro completely initialized.
    ACC_speed[LAT] = ACC_speed[LAT] - acc_south * tmp3;                // Positive when moving North cm/sec when no MAG this is speed to the front
    ACC_speed[LON] = ACC_speed[LON] - acc_west  * tmp3;                // Positive when moving East cm/sec when no MAG this is speed to the right
}

#ifdef BARO
///////////////////////////////////////////////
//Crashpilot1000 Mod getEstimatedAltitude ACC//
///////////////////////////////////////////////
#define VarioTabsize 8
#define BaroTabsize 5
void getEstimatedAltitude(void)
{
    static uint8_t  Vidx, Bidx, IniStep = 0, IniCnt;
    static float    BaroTab[BaroTabsize], VarioTab[VarioTabsize], LastEstAltBaro, SNRcorrect, SNRavg;
    float           BaroClimbRate, fltmp, EstAltBaro;
    uint8_t         i;

    if (!GroundAltInitialized)
    {
        if (newbaroalt)
        {
            switch(IniStep)                                      // Casemachine here for further extension
            {
            case 0:
                for (i = 0; i < VarioTabsize; i++) VarioTab[i] = 0;
                for (i = 0; i < BaroTabsize;  i++) BaroTab[i]  = 0;
                EstAlt = vario = SNRavg = 0;
                IniCnt = SonarStatus = 0;
                GroundAlt = BaroAlt;
                IniStep++;
                break;
            case 1:
                GroundAlt = GroundAlt * 0.8f + BaroAlt * 0.2f;
                IniCnt++;
                if (IniCnt == 100) GroundAltInitialized = true;  // Wait 100 * ca 27ms before proceeding
                break;
            }
        }
    }
    else
    {
        if (SonarStatus)                                         // Do sonar if available and everything is settled
        {
            if (SonarStatus == 1)
            {
                if (!SNRavg) SNRavg = (float)sonarAlt;
                else SNRavg = SNRavg * 0.8f + (float)sonarAlt * 0.2f; // Adjust Average during accepttimer (ca. 550ms so ca. 20 cycles)
                SNRcorrect = EstAlt + GroundAlt - SNRavg;        // Calculate baro/sonar displacement on 1st contact
            } else if (newbaroalt) BaroAlt = (SNRcorrect + (float)sonarAlt) * cfg.snr_cf + BaroAlt * (1 - cfg.snr_cf); // Set weight / make transition smoother
        } else SNRavg = 0;
        EstAlt += vario * ACCDeltaTimeINS;
        if (newbaroalt)                                          // MS Baro Timecheck 27ms // BMP085 Timecheck 26ms debug[0] = BaroDeltaTime/1000;
        {
            BaroTab[Bidx] = BaroAlt - GroundAlt; Bidx++;         // BaroAlt - GroundAlt Get EstAltBaro
            if (Bidx == BaroTabsize) Bidx = 0;
            fltmp = 0;
            for (i = 0; i < BaroTabsize; i++) fltmp += BaroTab[i];
            EstAltBaro     = fltmp / BaroTabsize;
            fltmp          = 1000000 / BaroDeltaTime;            // BaroDeltaTime in us
            VarioTab[Vidx] = (float)constrain(EstAltBaro - LastEstAltBaro, -127, 127) * fltmp;
            Vidx++;                                              // Baro Climbrate
            if (Vidx == VarioTabsize) Vidx = 0;
            LastEstAltBaro = EstAltBaro;
            fltmp = 0;
            for (i = 0; i < VarioTabsize; i++) fltmp += VarioTab[i];
            BaroClimbRate = fltmp / (float)VarioTabsize;         // BaroClimbRate in cm/sec // + is up // 27ms * 37 = 999ms
            vario  = vario  * cfg.accz_vcf + BaroClimbRate * (1.0f - cfg.accz_vcf);
            EstAlt = EstAlt * cfg.accz_acf + EstAltBaro    * (1.0f - cfg.accz_acf);
            if (cfg.bar_dbg)
            {
                debug[0] = EstAltBaro * 10;
                debug[1] = EstAlt * 10;
                debug[2] = BaroClimbRate;
                debug[3] = vario;
            }
        }
    }
}

void getAltitudePID(void)                                        // I put this out of getEstimatedAltitude seems logical
{
    float ThrAngle;
    ThrAngle = constrain(TiltValue * 100.0f, 0, 100.0f);
    BaroP  = BaroI = BaroD = 0;                                  // Reset the Pid, create something new, or not....
    if (ThrAngle < 40 || TiltValue < 0) return;                  // Don't do BaroPID if copter too tilted
    BaroP  = (int16_t)((float)cfg.P8[PIDALT] * (AltHold - EstAlt) * 0.005f);
    BaroI  = (int16_t)((float)cfg.I8[PIDALT] * vario * 0.02f);   // That is actually a "D"
    BaroD  = (int16_t)((float)cfg.D8[PIDALT] * (100.0f - ThrAngle) * 0.04f); // That is actually the Tiltcompensation
}
#endif

/*
    tmp0      = accLPFINS[0];
    tmp1      = accLPFINS[1];
    tmp2      = accLPFINS[2];
    tmp3      = sqrtf(tmp0 * tmp0 + tmp1 * tmp1 + tmp2 * tmp2);        // Normalize ACCvector so the gps ins works
    if (tmp3 == 0.0f) tmp3 = 1;                                        // In that case all tmp must be zero so div by 1 is ok
    tmp0      = tmp0 / tmp3;
    tmp1      = tmp1 / tmp3;
    tmp2      = tmp2 / tmp3;

float fsq(float x)
{
    return x * x;
}

float InvSqrt (float x)
{
    union
    {
        int32_t i;  
        float   f; 
    } conv;

    conv.f = x; 
    conv.i = 0x5f3759df - (conv.i >> 1); 
    return 0.5f * conv.f * (3.0f - x * conv.f * conv.f);
}

    tmp3      = InvSqrt(fsq(tmp0) + fsq(tmp1) + fsq(tmp2));            // Normalize acc vector
    tmp0      = tmp0 * tmp3;
    tmp1      = tmp1 * tmp3;
    tmp2      = tmp2 * tmp3;

float applyDeadbandFloat(float value, float deadband)
{
    if (abs(value) < deadband) value = 0;
     else if (value > 0) value = value - deadband;
           else if (value < 0) value = value + deadband;
    return value;
}

/////// GPS INS TESTCODE
//  Testcode
    static uint32_t previous5HzT = 0;
		flthead = 0;                                                // if no mag do bodyframe below
//  Testcode
    int16_t knob = constrain(rcData[AUX3]-1000,0,1000);
		float  knobbi= (float)knob * 0.001f;
		debug[0] = knobbi * 1000;
	  if (currentT > previous5HzT + 200000){
        previous5HzT = currentT;
		    VelNorth     = VelNorth * knobbi;
        VelEast      = VelEast  * knobbi;
		}
		debug[1] = VelNorth;
		debug[2] = VelEast;
/////// GPS INS TESTCODE
*/
