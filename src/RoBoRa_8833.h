/*
 * Copyright (c) 2025 Orazio Franco <robora2025@gmail.com>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#ifndef ROBORA_8833_H
#define ROBORA_8833_H

/*
 * DualDRV8833 - ESP32-C3 + Arduino Core 3.x
 * - 2 motori DC (A,B), ciascuno 2 pin + 2 canali LEDC
 * - setSpeed(-255..255), brake(), coast()
 * - Limite corrente opzionale con ADC (bang-bang + rampa)
 * - Deadzone, Expo%, Differential Mix (throttle+steer) opzionali
 *
 * NOTE LEDC 3.x:
 *   ledcAttachChannel(pin, freq, resBits, channel);
 *   ledcWrite(pin, duty);
 */

/* Includes ------------------------------------------------------------------*/
#include <Arduino.h>
/* Private includes ----------------------------------------------------------*/

/* Exported types ------------------------------------------------------------*/

/* Exported constants --------------------------------------------------------*/

/* Exported macro ------------------------------------------------------------*/

/* Exported functions prototypes ---------------------------------------------*/

/* Private defines -----------------------------------------------------------*/
#define ROBORA_SETCH_PWM /*Abilita la possibilità di selezionare anche il canale PWM */
#ifndef ROBORA_SETCH_PWM
#define ROBORA_A_CH_1 0
#define ROBORA_A_CH_2 1
#define ROBORA_B_CH_1 2
#define ROBORA_B_CH_2 3
#endif

#define ROBORA_CTRL_SLOW 0 /*Controllo lento*/
#define ROBORA_CTRL_FAST 1 /*Contrllo veloce*/

#define ROBORA_PWM_FCY_MIN 1000		 /*Frequency min*/
#define ROBORA_PWM_FCY_MAX 50000	 /*Frequency MAX*/
#define ROBORA_PWM_FCY_DEFAULT 20000 /*Frequency of PWM: 1 - 50000 Hz*/

#define ROBORA_PWM_RES_01bit 1						/* 1 Bit Resolution*/
#define ROBORA_PWM_RES_02bit 2						/* 2 Bit Resolution*/
#define ROBORA_PWM_RES_03bit 3						/* 3 Bit Resolution*/
#define ROBORA_PWM_RES_04bit 4						/* 4 Bit Resolution*/
#define ROBORA_PWM_RES_05bit 5						/* 5 Bit Resolution*/
#define ROBORA_PWM_RES_06bit 6						/* 6 Bit Resolution*/
#define ROBORA_PWM_RES_07bit 7						/* 7 Bit Resolution*/
#define ROBORA_PWM_RES_08bit 8						/* 8 Bit Resolution*/
#define ROBORA_PWM_RES_09bit 9						/* 9 Bit Resolution*/
#define ROBORA_PWM_RES_10bit 10						/*10 Bit Resolution*/
#define ROBORA_PWM_RES_11bit 11						/*11 Bit Resolution*/
#define ROBORA_PWM_RES_12bit 12						/*12 Bit Resolution*/
#define ROBORA_PWM_RES_13bit 13						/*13 Bit Resolution*/
#define ROBORA_PWM_RES_14bit 14						/*14 Bit Resolution*/
#define ROBORA_PWM_RES_DEFAULT ROBORA_PWM_RES_08bit /* Bit Resolution:   8-13 (5khz max 12bit resolution, 10khz to 30khz 10 bit resolturion,40khz to 50khz 9 bit resolution)*/

#define ROBORA_MAXVEL_DEFAULT 1.0f
#define ROBORA_STEERGAINPCT_DEFAULT 1.0f
#define ROBORA_ARCADELVL_DEFAULT 1.0f
#define ROBORA_EXPOPCT_DEFAULT 0.0f
#define ROBORA_DEADZONE_DEFAULT 0.05
#define ROBORA_ARCADEEN_DEFAULT 0

#define ROBORA_TANK_INVERTI_THROTTLE_DEFAULT 0
#define ROBORA_TANK_INVERTI_STEER_DEFAULT 0

class RoBoRa_8833
{
public:
#ifdef ROBORA_SETCH_PWM
	/*--- STRUCT CONFIG MOTOR ---*/
	struct MotorCfg
	{
		uint8_t in1;  // Pin comando 1
		uint8_t in2;  // Pin comando 2
		int8_t ch1;	  // Channel pwm per pin 1
		int8_t ch2;	  // Channel pwm per pin 2
		bool inverti; // Inversione
	};
#else
	/*--- STRUCT CONFIG MOTOR ---*/
	struct MotorCfg
	{
		uint8_t in1; // Pin comando 1
		uint8_t in2; // Pin comando 2
	};
#endif

	RoBoRa_8833(MotorCfg &a, MotorCfg &b, uint8_t FASTSLOWCTRL = ROBORA_CTRL_SLOW, uint32_t pwmFreq = ROBORA_PWM_FCY_DEFAULT, uint8_t pwmResBits = ROBORA_PWM_RES_DEFAULT)
		: A(a), B(b), FASTSLOWCTRL(), FREQ(pwmFreq), RESBITS(pwmResBits), MAX_DUTY((1u << pwmResBits) - 1), FirstStart(0) {}

	/*--- SETUP ---*/
	bool begin();

	/*--- PARAMETERI OPZIONALI GLOBALI ---*/
	void setMaxVel(uint8_t mv) { MaxVel = (float)mv / 100.0f; };			 // Massima velocita (100=100%)
	void setSteerGainPct(uint8_t st) { steerGainPct = (float)st / 100.0f; }; // Gain sullo sterzo nel mix differenziale (100 = 100%)
	void setDeadzone(uint8_t dz) { deadzone = (float)dz / 100.0f; };		 // Deadzone in counts(100 = 100%)
	void setExpoPct(uint8_t ex) { expoPct = (float)ex / 100.0f; };			 // Expo in percento (-100..+100).
	void setArcadeLvl(uint8_t lv) { arcadeLvl = (float)lv / 100.0f; };		 // Livello modalità arcade (100 = 100%)
	void setArcadeEn(uint8_t en) { arcadeEn = en; };						 // Livello modalità arcade Enable
	void setInvTankThr(uint8_t en) { invertiTankthrottle = en; };			 // Invereti controllo throttle in modalità tank
	void setInvTankStr(uint8_t en) { invertiTankSteer = en; };					 // Invereti controllo steer in modalità tank

	/*--- PARAMETERI OPZIONALI SINGLE MOTOR ---*/
	void setInvertiA(uint8_t en) { A.inverti = en; }; // Invereti controllo motore A
	void setInvertiB(uint8_t en) { B.inverti = en; }; // Invereti controllo motore A

	/*--- CONTROLLO DIRETTO MOTORI ---*/
	void brakeA();														 // Frenata A
	void brakeB();														 // Frenata B
	void brakeAll();													 // Frenata A+A
	void coastA();														 // Folle A
	void coastB();														 // Folle B
	void coastAll();													 // Folle A+B
	void setSpeedA(int16_t s);											 // s ∈ [-255..255]
	void setSpeedB(int16_t s);											 // s ∈ [-255..255]
	void driveTank(int16_t throttle, int16_t steer, uint16_t max = 127); // Differential mixing (tank):  throttle, steer ∈ [-127..127]

	/*--- FOR INFO ---*/
	uint32_t getLastTargtA(void) { return LastTargtA; }; // Last value of target Motor A
	uint32_t getLastTargtB(void) { return LastTargtB; }; // Last value of target Motor B
	void printConfig(void);								 /*Stampa tutta la configuazione */

private:
	/*--- PARAMETERI ---*/
	MotorCfg A, B; // Configurazione motori
	uint8_t FirstStart = 0;
	uint8_t FASTSLOWCTRL; //  Fast or slow Decay
	uint32_t FREQ;
	uint8_t RESBITS;
	uint32_t MAX_DUTY;
	uint32_t LastTargtA;
	uint32_t LastTargtB;

	/*--- FUNZIONI GESTIONE PIN MOTORE ---*/
	inline void _writePins(MotorCfg &m, uint32_t d1, uint32_t d2);
	inline void _coast(MotorCfg &m);
	inline void _brake(MotorCfg &m);

	/*--- CORRETTORI VELOCITA' ---*/
	float MaxVel = ROBORA_MAXVEL_DEFAULT;			  //
	float steerGainPct = ROBORA_STEERGAINPCT_DEFAULT; //
	float deadzone = ROBORA_DEADZONE_DEFAULT;		  //
	float expoPct = ROBORA_EXPOPCT_DEFAULT;			  //
	float arcadeLvl = ROBORA_ARCADELVL_DEFAULT;		  //
	uint8_t arcadeEn = ROBORA_ARCADEEN_DEFAULT;		  //

	uint8_t invertiTankthrottle = ROBORA_TANK_INVERTI_THROTTLE_DEFAULT;
	uint8_t invertiTankSteer = ROBORA_TANK_INVERTI_STEER_DEFAULT;

	static inline float applyDeadzone(float v, float dz); // Dead zone
	static inline float applyExpo(float v, float e);	  // expo blend: out = (1-e)*v + e*v^3

	// Applica comando “shapato” a un motore
	void _cmdMotor(MotorCfg &m, int16_t s_raw);
	inline void _applyOutput(MotorCfg &m, int16_t speed, uint32_t duty);
};

#endif /*ROBORA_8833_H*/
