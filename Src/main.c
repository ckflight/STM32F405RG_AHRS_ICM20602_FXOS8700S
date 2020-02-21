

#include "stm32f4xx.h"

#include "CK_SYSTEM.h"
#include "CK_UART.h"
#include "CK_TIME_HAL.h"
#include "CK_I2C.h"
#include "CK_SPI.h"
#include "math.h"

#include "USBD_CDC/CK_USBD_INTERFACE.h"

#define FXAS21002C_ADDRESS             0x21
#define FXOS8700_ADDRESS               0x1F

#define M_PIf        				 3.14159265358979323846f

SPI_TypeDef * SPI_ICM20602 = SPI1;
GPIO_TypeDef* GPIO_CS_ICM20602 = GPIOA;
uint8_t CS_PIN_ICM20602 = 4;

// Offsets applied to raw x/y/z values
float mag_offsets[3]            = { 18.92F, -9.66F, 55.33F };

// Soft iron error compensation matrix
float mag_softiron_matrix[3][3] = { {  0.982,  -0.004,  0.006 },
                                    { -0.004,   0.992,  0.003 },
                                    {  0.006,   0.003,  1.027 } };

float mag_field_strength        = 42.43F;

int16_t gyroRaw[3];
int32_t gyroCalib[3];
float gyroFinal[3];
float gyroFiltered[3];
float gyroSensitivity;

int16_t accRaw[3];
int32_t accCalib[3];
float accFinal[3];
float accFiltered[3];
float accSensitivity;

int16_t magRaw[3];
float magGauss[3];
float magMicroTesla[3];
float magFinal[3];
float magFiltered[3];
float mag_heading;

uint8_t i2cdata[20];

void initSensor();

void readGyro(void);
void updateGyro(void);

void readAcc(void);
void updateAcc(void);

void readMag(void);
void updateMag(void);

void CK_IMU_MadwickUpdate2();

float CK_MATH_invSqrt(float x);
void CK_IMU_ComputeEulerAngles();
void printResults(float roll, float pitch, float heading);
void calibrateAccGyro(void);
void calculateHeading();

float beta = 0.1f;
float q0 = 1.0f;
float q1 = 0.0f;
float q2 = 0.0f;
float q3 = 0.0f;

float imu_dT = 1.0f / 100.0f;
float roll;
float pitch;
float yaw;

#define USB_DEBUG
//#define UART_DEBUG

//#define GYRO_DEBUG
//#define ACC_DEBUG
#define MAG_DEBUG
//#define IMU_DEBUG

uint32_t loopTimer;
uint32_t loopTime = 10000; //100Hz
int imuSyncCounter = 0;
int main(void){

	CK_SYSTEM_SetSystemClock(SYSTEM_CLK_168MHz);

	HAL_Init();

	CK_USBD_Init();

	CK_SPI_Init(SPI_ICM20602);

	CK_I2C_Init(I2C1, CK_I2C_400Khz);


#ifdef USB_DEBUG
	CK_USBD_StringPrintln("AHRS_FXOS8700C_ICM20602");
	CK_USBD_Transmit();
#else
	CK_USBD_StringPrintln("AHRS_FXOS8700C_ICM20602");
#endif

	initSensor();

	calibrateAccGyro();

	loopTimer = CK_microSec();

	while(1){

		//uint32_t a1 = CK_microSec();

		updateGyro();
		updateAcc();
		updateMag();

		CK_IMU_MadwickUpdate2();

		calculateHeading();

		printResults((int)roll, (int)pitch, (int)(360.0f - yaw));

#ifdef USB_DEBUG

		//CK_USBD_StringPrint("TIME");CK_USBD_IntPrintln(a2);
		//CK_USBD_DEVICE_Transmit(CK_USB_FS);
#else
		//uint32_t a2 = CK_microSec() - a1;
		//CK_USBD_StringPrint("TIME:");CK_USBD_IntPrintln(a2);
#endif


		while((CK_microSec() - loopTimer) < loopTime);
		loopTimer = CK_microSec();

	}
}

void initSensor(){

	// ICM20602

	//ICM20602 CS Pin
	CK_GPIOx_ClockEnable(GPIO_CS_ICM20602);

	CK_GPIOx_Init(GPIO_CS_ICM20602, CS_PIN_ICM20602, CK_GPIO_OUTPUT, CK_GPIO_NOAF, CK_GPIO_PUSHPULL, CK_GPIO_VERYHIGH, CK_GPIO_NOPUPD);

	CK_GPIOx_SetPin(GPIO_CS_ICM20602, CS_PIN_ICM20602);//Set CS High for Idle

	if(CK_SPI_WriteRegister(0x75|0x80, 0xFF, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602) == 0x12){

		//ICM20602 Setup
		CK_delayMs(100);//First Boot Wait
		CK_SPI_WriteRegister(0x70, 1u<<6, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);//I2C DISABLE,SPI ONLY
		CK_delayMs(30);

		CK_SPI_WriteRegister(0x6B, 1u<<7, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);//DEVICE RESET
		CK_delayMs(100);

		CK_SPI_WriteRegister(0x68, 0x03, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);//ACCEL,TEMP SIGNAL PATH RESET
		CK_delayMs(30);

		CK_SPI_WriteRegister(0x6A, 0x05, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);//RESET REGISTERS
		CK_delayMs(100);

		CK_SPI_WriteRegister(0x6B, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);//CLEAR PWR MNG1 REGISTER
		CK_delayMs(30);

		CK_SPI_WriteRegister(0x6B, 0x01, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);//CLK SLECTION FOR BEST GYRO PERFORMANCE
		CK_delayMs(30);

		CK_SPI_WriteRegister(0x19, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);//SMPL RATE DIV = 0 8Khz/1+SMPL RATE DIV = 8Khz
		CK_delayMs(30);

		CK_SPI_WriteRegister(0x1A, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);//DLPF_CFG[00] 8KHz, Bit 7 set to 0
		CK_delayMs(30);

		CK_SPI_WriteRegister(0x1B, 0x18, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);//FCHOICE_B[00], FSEL[11]CK_ICM20602_DPS2000
		CK_delayMs(30);

		CK_SPI_WriteRegister(0x1C, 0x18, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);//ACCEL_FS_SEL[11] +-16g 0x18, 2g 0x00
		CK_delayMs(30);

		CK_SPI_WriteRegister(0x1D, 0x05, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602);//ACCEL_FCHOICE_B=0,A_DLPF_CFG=0 1KHz ACCEL
		CK_delayMs(30);

	}


	CK_I2C_ReadMulti(FXAS21002C_ADDRESS, 0x0C, i2cdata, 1);
	uint8_t id = i2cdata[0];
	CK_USBD_IntPrintln(id);

	if(id != 0xD7){
#ifdef USB_DEBUG
	CK_USBD_StringPrintln("ERROR");
	CK_USBD_Transmit();
#else
	CK_USBD_StringPrintln("ERROR");
#endif

		while(1);
	}
	else{
#ifdef USB_DEBUG
		CK_USBD_StringPrintln("GYRO FOUND");
		CK_USBD_Transmit();
#else
		CK_USBD_StringPrintln("GYRO FOUND");
#endif
	}

	CK_I2C_ReadMulti(FXOS8700_ADDRESS, 0x0D, i2cdata, 1);
	id = i2cdata[0];
	CK_USBD_IntPrintln(id);

	if(id != 0xC7){
#ifdef USB_DEBUG
		CK_USBD_StringPrintln("ERROR");
		CK_USBD_Transmit();
#else
		CK_USBD_StringPrintln("ERROR");
#endif
		while(1);
	}
	else{
#ifdef USB_DEBUG
		CK_USBD_StringPrintln("ACC FOUND");
		CK_USBD_Transmit();
#else
		CK_USBD_StringPrintln("ACC FOUND");
#endif
	}

	CK_I2C_Transfer(FXAS21002C_ADDRESS, 0x13, 0x00);// CTRL_REG1, STANDBY
	CK_I2C_Transfer(FXAS21002C_ADDRESS, 0x13, 1u<<6);// CTRL_REG1, RESET
	CK_I2C_Transfer(FXAS21002C_ADDRESS, 0x0D, 0x00); // CTRL_REG0, 2000deg/sec, 62.5mdps/digit
	gyroSensitivity = 0.0625f;
	CK_I2C_Transfer(FXAS21002C_ADDRESS, 0x13, 0x0E); // CTRL_REG1, ACTIVE 100Hz, 0x02 is 800HZ ODR,
	CK_delayMs(100);

	// FXOS8700
	//ACC
	CK_I2C_Transfer(FXOS8700_ADDRESS, 0x2A, 0x00);    // CTRL_REG1, STANDBY
	CK_I2C_Transfer(FXOS8700_ADDRESS, 0x0E, 0x01); // XYZ_DATA_CFG, +-4g 0.488 mg/LSB
	accSensitivity = 0.000488f;
	CK_I2C_Transfer(FXOS8700_ADDRESS, 0x2B, 0x02); // CTRL_REG2, High Resolution Mode
	CK_I2C_Transfer(FXOS8700_ADDRESS, 0x2A, 0x15); // CTRL_REG1, Hybrid 100Hz, Low Noise, Active

	//MAG
	CK_I2C_Transfer(FXOS8700_ADDRESS, 0x5B, 0x1F); // M_CTRL_REG1, Oversampling 16 100Hz hybrind, acc mag active
	CK_I2C_Transfer(FXOS8700_ADDRESS, 0x5C, 0x20); // M_CTRL_REG2, Jump to reg 0x33 after reading 0x06

}

void readGyro(void){

	gyroRaw[0] = (int16_t)(CK_SPI_WriteRegister(0x43|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602) <<8 | CK_SPI_WriteRegister(0x44|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602));

	gyroRaw[1] = (int16_t)(CK_SPI_WriteRegister(0x45|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602) <<8 | CK_SPI_WriteRegister(0x46|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602));

	gyroRaw[2] = (int16_t)(CK_SPI_WriteRegister(0x47|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602) <<8 | CK_SPI_WriteRegister(0x48|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602));

}

void updateGyro(void){

	readGyro();

	gyroRaw[0] -= gyroCalib[0];
	gyroRaw[1] -= gyroCalib[1];
	gyroRaw[2] -= gyroCalib[2];

	gyroFinal[0] = (float)gyroRaw[0] * gyroSensitivity;
	gyroFinal[1] = (float)gyroRaw[1] * gyroSensitivity;
	gyroFinal[2] = (float)gyroRaw[2] * gyroSensitivity;

	gyroFiltered[0] = gyroFiltered[0] * 0.8 + gyroFinal[0] * 0.2;
	gyroFiltered[1] = gyroFiltered[1] * 0.8 + gyroFinal[1] * 0.2;
	gyroFiltered[2] = gyroFiltered[2] * 0.8 + gyroFinal[2] * 0.2;

}

void readAcc(void){

	accRaw[0] = (int16_t)(CK_SPI_WriteRegister(0x3B|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602) <<8 | CK_SPI_WriteRegister(0x3C|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602));

	accRaw[1] = (int16_t)(CK_SPI_WriteRegister(0x3D|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602) <<8 | CK_SPI_WriteRegister(0x3E|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602));

	accRaw[2] = (int16_t)(CK_SPI_WriteRegister(0x3F|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602) <<8 | CK_SPI_WriteRegister(0x40|0x80, 0, SPI_ICM20602, GPIO_CS_ICM20602, CS_PIN_ICM20602));

}

void updateAcc(void){

	readAcc();

	accRaw[0] -= accCalib[0];
	accRaw[1] -= accCalib[1];
	//accRaw[2] -= accCalib[2];// leave 1 g

	// g value
	accFinal[0] = (float)accRaw[0] * accSensitivity;
	accFinal[1] = (float)accRaw[1] * accSensitivity;
	accFinal[2] = (float)accRaw[2] * accSensitivity;

	accFiltered[0] = accFiltered[0] * 0.8 + accFinal[0] * 0.2;
	accFiltered[1] = accFiltered[1] * 0.8 + accFinal[1] * 0.2;
	accFiltered[2] = accFiltered[2] * 0.8 + accFinal[2] * 0.2;

}

void readMag(){

	// Ignoe acc reading i get it from icm20602

	// M_OUT_X_MSB 0x33
	CK_I2C_ReadMulti(FXOS8700_ADDRESS, 0x33, i2cdata, 6);

	uint8_t mxhi = i2cdata[0];
	uint8_t mxlo = i2cdata[1];
	uint8_t myhi = i2cdata[2];
	uint8_t mylo = i2cdata[3];
	uint8_t mzhi = i2cdata[4];
	uint8_t mzlo = i2cdata[5];

	magRaw[0] = (int16_t)((mxhi << 8) | mxlo);
	magRaw[1] = (int16_t)((myhi << 8) | mylo);
	magRaw[2] = (int16_t)((mzhi << 8) | mzlo);

}

void updateMag(void){

	readMag();

	// 0.1 microTesla/LSB for this sensor
	magMicroTesla[0] =  (float)magRaw[0] * 0.1f;
	magMicroTesla[1] =  (float)magRaw[1] * 0.1f;
	magMicroTesla[2] =  (float)magRaw[2] * 0.1f;

	// 1 Gauss 100 microTesla
	magGauss[0] = magMicroTesla[0] / 100.0f;
	magGauss[1] = magMicroTesla[1] / 100.0f;
	magGauss[2] = magMicroTesla[2] / 100.0f;

	float x = magMicroTesla[0] - mag_offsets[0];
	float y = magMicroTesla[1] - mag_offsets[1];
	float z = magMicroTesla[2] - mag_offsets[2];

	// Apply mag soft iron error compensation
	magFinal[0] = x * mag_softiron_matrix[0][0] + y * mag_softiron_matrix[0][1] + z * mag_softiron_matrix[0][2];
	magFinal[1] = x * mag_softiron_matrix[1][0] + y * mag_softiron_matrix[1][1] + z * mag_softiron_matrix[1][2];
	magFinal[2] = x * mag_softiron_matrix[2][0] + y * mag_softiron_matrix[2][1] + z * mag_softiron_matrix[2][2];

	//magFiltered[0] = magFiltered[0] * 0.8 + magFinal[0] * 0.2;
	//magFiltered[1] = magFiltered[1] * 0.8 + magFinal[1] * 0.2;
	//magFiltered[2] = magFiltered[2] * 0.8 + magFinal[2] * 0.2;

}

void CK_IMU_MadwickUpdate2(void){

	imuSyncCounter++;

	if(imuSyncCounter >= 0){

		imuSyncCounter = 0;

		float recipNorm;
		float s0, s1, s2, s3;
		float qDot1, qDot2, qDot3, qDot4;
		float hx, hy;
		float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3, _2q0q2, _2q2q3, q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;

		float gx = gyroFiltered[0];
		float gy = gyroFiltered[1];
		float gz = gyroFiltered[2];

		float ax = accFiltered[0];
		float ay = accFiltered[1];
		float az = accFiltered[2];

		float mx = magFiltered[0];
		float my = magFiltered[1];
		float mz = magFiltered[2];

		// Convert gyroscope degrees/sec to radians/sec
		gx *= 0.0174533f;
		gy *= 0.0174533f;
		gz *= 0.0174533f;

		// Rate of change of quaternion from gyroscope
		qDot1 = 0.5f * (-q1 * gx - q2 * gy - q3 * gz);
		qDot2 = 0.5f * (q0 * gx + q2 * gz - q3 * gy);
		qDot3 = 0.5f * (q0 * gy - q1 * gz + q3 * gx);
		qDot4 = 0.5f * (q0 * gz + q1 * gy - q2 * gx);

		// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
		if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

			// Normalise accelerometer measurement
			recipNorm = CK_MATH_invSqrt(ax * ax + ay * ay + az * az);
			ax *= recipNorm;
			ay *= recipNorm;
			az *= recipNorm;

			// Normalise magnetometer measurement
			recipNorm = CK_MATH_invSqrt(mx * mx + my * my + mz * mz);
			mx *= recipNorm;
			my *= recipNorm;
			mz *= recipNorm;

			// Auxiliary variables to avoid repeated arithmetic
			_2q0mx = 2.0f * q0 * mx;
			_2q0my = 2.0f * q0 * my;
			_2q0mz = 2.0f * q0 * mz;
			_2q1mx = 2.0f * q1 * mx;
			_2q0 = 2.0f * q0;
			_2q1 = 2.0f * q1;
			_2q2 = 2.0f * q2;
			_2q3 = 2.0f * q3;
			_2q0q2 = 2.0f * q0 * q2;
			_2q2q3 = 2.0f * q2 * q3;
			q0q0 = q0 * q0;
			q0q1 = q0 * q1;
			q0q2 = q0 * q2;
			q0q3 = q0 * q3;
			q1q1 = q1 * q1;
			q1q2 = q1 * q2;
			q1q3 = q1 * q3;
			q2q2 = q2 * q2;
			q2q3 = q2 * q3;
			q3q3 = q3 * q3;

			// Reference direction of Earth's magnetic field
			hx = mx * q0q0 - _2q0my * q3 + _2q0mz * q2 + mx * q1q1 + _2q1 * my * q2 + _2q1 * mz * q3 - mx * q2q2 - mx * q3q3;
			hy = _2q0mx * q3 + my * q0q0 - _2q0mz * q1 + _2q1mx * q2 - my * q1q1 + my * q2q2 + _2q2 * mz * q3 - my * q3q3;
			_2bx = sqrtf(hx * hx + hy * hy);
			_2bz = -_2q0mx * q2 + _2q0my * q1 + mz * q0q0 + _2q1mx * q3 - mz * q1q1 + _2q2 * my * q3 - mz * q2q2 + mz * q3q3;
			_4bx = 2.0f * _2bx;
			_4bz = 2.0f * _2bz;

			// Gradient decent algorithm corrective step
			s0 = -_2q2 * (2.0f * q1q3 - _2q0q2 - ax) + _2q1 * (2.0f * q0q1 + _2q2q3 - ay) - _2bz * q2 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q3 + _2bz * q1) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q2 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s1 = _2q3 * (2.0f * q1q3 - _2q0q2 - ax) + _2q0 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q1 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + _2bz * q3 * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q2 + _2bz * q0) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q3 - _4bz * q1) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s2 = -_2q0 * (2.0f * q1q3 - _2q0q2 - ax) + _2q3 * (2.0f * q0q1 + _2q2q3 - ay) - 4.0f * q2 * (1 - 2.0f * q1q1 - 2.0f * q2q2 - az) + (-_4bx * q2 - _2bz * q0) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (_2bx * q1 + _2bz * q3) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + (_2bx * q0 - _4bz * q2) * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			s3 = _2q1 * (2.0f * q1q3 - _2q0q2 - ax) + _2q2 * (2.0f * q0q1 + _2q2q3 - ay) + (-_4bx * q3 + _2bz * q1) * (_2bx * (0.5f - q2q2 - q3q3) + _2bz * (q1q3 - q0q2) - mx) + (-_2bx * q0 + _2bz * q2) * (_2bx * (q1q2 - q0q3) + _2bz * (q0q1 + q2q3) - my) + _2bx * q1 * (_2bx * (q0q2 + q1q3) + _2bz * (0.5f - q1q1 - q2q2) - mz);
			recipNorm = CK_MATH_invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
			s0 *= recipNorm;
			s1 *= recipNorm;
			s2 *= recipNorm;
			s3 *= recipNorm;

			// Apply feedback step
			qDot1 -= beta * s0;
			qDot2 -= beta * s1;
			qDot3 -= beta * s2;
			qDot4 -= beta * s3;
		}

		// Integrate rate of change of quaternion to yield quaternion
		q0 += qDot1 * imu_dT;
		q1 += qDot2 * imu_dT;
		q2 += qDot3 * imu_dT;
		q3 += qDot4 * imu_dT;

		// Normalise quaternion
		recipNorm = CK_MATH_invSqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
		q0 *= recipNorm;
		q1 *= recipNorm;
		q2 *= recipNorm;
		q3 *= recipNorm;

		CK_IMU_ComputeEulerAngles();

	}

}


void CK_IMU_ComputeEulerAngles(void){

	// Compute Roll, Pitch Yaw Angles
	roll  = atan2f(q0*q1 + q2*q3, 0.5f - q1*q1 - q2*q2);
	pitch = asinf(-2.0f * (q1*q3 - q0*q2));
	yaw   = atan2f(q1*q2 + q0*q3, 0.5f - q2*q2 - q3*q3);

	roll  *= 57.29578f;
	pitch *= 57.29578f;
	yaw   *= 57.29578f;
	yaw   +=  180.0f;

}

float CK_MATH_invSqrt(float x){

	float halfx = 0.5f * x;
	float y = x;
	long i = *(long*)&y;
	i = 0x5f3759df - (i>>1);
	y = *(float*)&i;
	y = y * (1.5f - (halfx * y * y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

void printResults(float roll, float pitch, float heading){

#ifdef USB_DEBUG

#ifdef GYRO_DEBUG
	CK_USBD_IntPrint(gyroRaw[0]);CK_USBD_StringPrint("\t");
	CK_USBD_IntPrint(gyroRaw[1]);CK_USBD_StringPrint("\t");
	CK_USBD_IntPrint(gyroRaw[2]);CK_USBD_StringPrint("\t");

	CK_USBD_FloatPrint(gyroFinal[0]);CK_USBD_StringPrint("\t");
	CK_USBD_FloatPrint(gyroFinal[1]);CK_USBD_StringPrint("\t");
	CK_USBD_FloatPrint(gyroFinal[2]);CK_USBD_StringPrint("\t");

	CK_USBD_FloatPrint(gyroFiltered[0]);CK_USBD_StringPrint("\t");
	CK_USBD_FloatPrint(gyroFiltered[1]);CK_USBD_StringPrint("\t");
	CK_USBD_FloatPrintln(gyroFiltered[2]);

	CK_USBD_Transmit();

#endif

#ifdef ACC_DEBUG
	CK_USBD_IntPrint(accRaw[0]);CK_USBD_StringPrint("\t");
	CK_USBD_IntPrint(accRaw[1]);CK_USBD_StringPrint("\t");
	CK_USBD_IntPrint(accRaw[2]);CK_USBD_StringPrint("\t");

	CK_USBD_FloatPrint(accFinal[0]);CK_USBD_StringPrint("\t");
	CK_USBD_FloatPrint(accFinal[1]);CK_USBD_StringPrint("\t");
	CK_USBD_FloatPrint(accFinal[2]);CK_USBD_StringPrint("\t");

	CK_USBD_FloatPrint(accFiltered[0]);CK_USBD_StringPrint("\t");
	CK_USBD_FloatPrint(accFiltered[1]);CK_USBD_StringPrint("\t");
	CK_USBD_FloatPrintln(accFiltered[2]);

	CK_USBD_Transmit();
#endif

#ifdef MAG_DEBUG
	CK_USBD_IntPrint(magRaw[0]);CK_USBD_StringPrint(" ");
	CK_USBD_IntPrint(magRaw[1]);CK_USBD_StringPrint(" ");
	CK_USBD_IntPrint(magRaw[2]);CK_USBD_StringPrint(" ");

	CK_USBD_FloatPrint(magGauss[0]);CK_USBD_StringPrint(" ");
	CK_USBD_FloatPrint(magGauss[1]);CK_USBD_StringPrint(" ");
	CK_USBD_FloatPrint(magGauss[2]);CK_USBD_StringPrint(" ");

	CK_USBD_FloatPrint(magMicroTesla[0]);CK_USBD_StringPrint(" ");
	CK_USBD_FloatPrint(magMicroTesla[1]);CK_USBD_StringPrint(" ");
	CK_USBD_FloatPrint(magMicroTesla[2]);CK_USBD_StringPrint(" ");

	CK_USBD_FloatPrint(magFinal[0]);CK_USBD_StringPrint(" ");
	CK_USBD_FloatPrint(magFinal[1]);CK_USBD_StringPrint(" ");
	CK_USBD_FloatPrint(magFinal[2]);CK_USBD_StringPrint(" ");

	CK_USBD_FloatPrint(magFiltered[0]);CK_USBD_StringPrint(" ");
	CK_USBD_FloatPrint(magFiltered[1]);CK_USBD_StringPrint(" ");
	CK_USBD_FloatPrintln(magFiltered[2]);

	CK_USBD_Transmit();

#endif

#ifdef IMU_DEBUG
	CK_USBD_FloatPrint(roll);CK_USBD_StringPrint("\t");
	CK_USBD_FloatPrint(pitch);CK_USBD_StringPrint("\t");
	CK_USBD_FloatPrintln(heading);

	CK_USBD_Transmit();
#endif

#endif

#ifdef UART_DEBUG

#ifdef GYRO_DEBUG
	CK_USBD_IntPrint(gyroRaw[0]);CK_USBD_StringPrint("\t");
	CK_USBD_IntPrint(gyroRaw[1]);CK_USBD_StringPrint("\t");
	CK_USBD_IntPrint(gyroRaw[2]);CK_USBD_StringPrint("\t");

	CK_FloatPrint(gyroFinal[0]);CK_USBD_StringPrint("\t");
	CK_FloatPrint(gyroFinal[1]);CK_USBD_StringPrint("\t");
	CK_FloatPrint(gyroFinal[2]);CK_USBD_StringPrint("\t");

	CK_FloatPrint(gyroFiltered[0]);CK_USBD_StringPrint("\t");
	CK_FloatPrint(gyroFiltered[1]);CK_USBD_StringPrint("\t");
	CK_FloatPrintln(gyroFiltered[2]);

#endif

#ifdef ACC_DEBUG
	CK_USBD_IntPrint(accRaw[0]);CK_USBD_StringPrint("\t");
	CK_USBD_IntPrint(accRaw[1]);CK_USBD_StringPrint("\t");
	CK_USBD_IntPrint(accRaw[2]);CK_USBD_StringPrint("\t");

	CK_FloatPrint(accFinal[0]);CK_USBD_StringPrint("\t");
	CK_FloatPrint(accFinal[1]);CK_USBD_StringPrint("\t");
	CK_FloatPrint(accFinal[2]);CK_USBD_StringPrint("\t");

	CK_FloatPrint(accFiltered[0]);CK_USBD_StringPrint("\t");
	CK_FloatPrint(accFiltered[1]);CK_USBD_StringPrint("\t");
	CK_FloatPrintln(accFiltered[2]);
#endif

#ifdef MAG_DEBUG
	CK_USBD_IntPrint(magRaw[0]);CK_USBD_StringPrint(" ");
	CK_USBD_IntPrint(magRaw[1]);CK_USBD_StringPrint(" ");
	CK_USBD_IntPrint(magRaw[2]);CK_USBD_StringPrint(" ");

	CK_FloatPrint(magGauss[0]);CK_USBD_StringPrint(" ");
	CK_FloatPrint(magGauss[1]);CK_USBD_StringPrint(" ");
	CK_FloatPrint(magGauss[2]);CK_USBD_StringPrint(" ");

	CK_FloatPrint(magMicroTesla[0]);CK_USBD_StringPrint(" ");
	CK_FloatPrint(magMicroTesla[1]);CK_USBD_StringPrint(" ");
	CK_FloatPrint(magMicroTesla[2]);CK_USBD_StringPrint(" ");

	CK_FloatPrint(magFinal[0]);CK_USBD_StringPrint(" ");
	CK_FloatPrint(magFinal[1]);CK_USBD_StringPrint(" ");
	CK_FloatPrint(magFinal[2]);CK_USBD_StringPrint(" ");

	CK_FloatPrint(magFiltered[0]);CK_USBD_StringPrint(" ");
	CK_FloatPrint(magFiltered[1]);CK_USBD_StringPrint(" ");
	CK_FloatPrintln(magFiltered[2]);

#endif

#ifdef IMU_DEBUG
	CK_FloatPrint(roll);CK_USBD_StringPrint("\t");
	CK_FloatPrint(pitch);CK_USBD_StringPrint("\t");
	CK_FloatPrintln(heading);
#endif

#endif

}


void calibrateAccGyro(void){

	int numOfSamples = 100;
	gyroCalib[0] = 0;
	gyroCalib[1] = 0;
	gyroCalib[2] = 0;

#ifdef USB_DEBUG
	CK_USBD_StringPrint("Gyro Calibration");
	CK_USBD_Transmit();
#else
	CK_USBD_StringPrint("Gyro Calibration");
#endif


	for(int i = 0; i < numOfSamples; i++){
		readGyro();
		gyroCalib[0] += gyroRaw[0];
		gyroCalib[1] += gyroRaw[1];
		gyroCalib[2] += gyroRaw[2];

		CK_delayMs(10);
		if(i%10 == 1){
#ifdef USB_DEBUG
			CK_USBD_StringPrint(".");
			CK_USBD_Transmit();
#else
			CK_USBD_StringPrint(".");
#endif
		}
	}

	gyroCalib[0] /= numOfSamples;
	gyroCalib[1] /= numOfSamples;
	gyroCalib[2] /= numOfSamples;
#ifdef USB_DEBUG
	CK_USBD_StringPrintln("");
	CK_USBD_Transmit();
#else
	CK_USBD_StringPrintln("");
#endif

	accCalib[0] = 0;
	accCalib[1] = 0;
	accCalib[2] = 0;

#ifdef USB_DEBUG
	CK_USBD_StringPrint("Acc Calibration");
	CK_USBD_Transmit();
#else
	CK_USBD_StringPrint("Acc Calibration");
#endif
	for(int i = 0; i < numOfSamples; i++){
		readAcc();
		accCalib[0] += accRaw[0];
		accCalib[1] += accRaw[1];
		accCalib[2] += accRaw[2];

		CK_delayMs(10);
		if(i%10 == 1){
#ifdef USB_DEBUG
			CK_USBD_StringPrint(".");
			CK_USBD_Transmit();
#else
			CK_USBD_StringPrint(".");
#endif
		}
	}

	accCalib[0] /= numOfSamples;
	accCalib[1] /= numOfSamples;
	accCalib[2] /= numOfSamples;
#ifdef USB_DEBUG
	CK_USBD_StringPrintln("");
	CK_USBD_Transmit();
#else
	CK_USBD_StringPrintln("");
#endif


#ifdef USB_DEBUG
	CK_USBD_IntPrint(gyroCalib[0]);CK_USBD_StringPrint(",");
	CK_USBD_IntPrint(gyroCalib[1]);CK_USBD_StringPrint(",");
	CK_USBD_IntPrint(gyroCalib[2]);
	CK_USBD_StringPrint("    ");
	CK_USBD_IntPrint(accCalib[0]);CK_USBD_StringPrint(",");
	CK_USBD_IntPrint(accCalib[1]);CK_USBD_StringPrint(",");
	CK_USBD_IntPrintln(accCalib[2]);
	CK_USBD_Transmit();
#else
	CK_USBD_IntPrint(gyroCalib[0]);CK_USBD_StringPrint(",");
	CK_USBD_IntPrint(gyroCalib[1]);CK_USBD_StringPrint(",");
	CK_USBD_IntPrint(gyroCalib[2]);
	CK_USBD_StringPrint("    ");
	CK_USBD_IntPrint(accCalib[0]);CK_USBD_StringPrint(",");
	CK_USBD_IntPrint(accCalib[1]);CK_USBD_StringPrint(",");
	CK_USBD_IntPrintln(accCalib[2]);
#endif


}

void calculateHeading(){

	float mx = magFiltered[0];
	float my = magFiltered[1];
	float mz = magFiltered[2];

	// axis swapped in 9dof breakout
	float roll_rad = roll / 57.29578f; //in radians/s
	float pitch_rad = pitch / 57.29578f; //in radians/s

	float mx_compensated = (mx * cosf(pitch_rad)) + (mz * sinf(pitch_rad));
	float my_compensated = (mx * sinf(roll_rad) * sinf(pitch_rad)) + (my * cosf(roll_rad)) - (mz * sinf(roll_rad) * cosf(pitch_rad));

	float magX = mx_compensated;
	float magY = my_compensated;

	float heading = 0;

	if(magX == 0){
		if(magY < 0){
		  heading = 90;
		}
		else if(magY >= 0){
		  heading = 0;
		}
	}
	else if(magX != 0){
		// Find here: http://www.magnetic-declination.com/
		// Magnetic declination: 5° 41' EAST (POSITIVE);  1 degreee = 0.0174532925 radians
		#define DEC_ANGLE 0.099
		heading = atan2(magY, magX);
		heading += DEC_ANGLE; // subtraction works not adding
		heading *= (180.0f / M_PIf);
	}

	if(heading > 360.0f){
		heading -= 360.0f;
	}
	if(heading < 0.0f){
		heading += 360.0f;
	}

	mag_heading = (int)heading;

}


