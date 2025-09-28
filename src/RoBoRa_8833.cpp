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


#include "RoBoRa_8833.h"

// ---- SETUP ---- //
bool RoBoRa_8833::begin()
{
    bool ok = true;

    if (FirstStart) /**/
    {
        ledcDetach(A.in1);
        ledcDetach(A.in2);
        ledcDetach(B.in1);
        ledcDetach(B.in2);
    }
#ifdef ROBORA_SETCH_PWM

    ok &= ledcAttachChannel(A.in1, FREQ, RESBITS, A.ch1);
    ok &= ledcAttachChannel(A.in2, FREQ, RESBITS, A.ch2);
    ok &= ledcAttachChannel(B.in1, FREQ, RESBITS, B.ch1);
    ok &= ledcAttachChannel(B.in2, FREQ, RESBITS, B.ch2);
#else
    ok &= ledcAttachChannel(A.in1, FREQ, RESBITS, ROBORA_A_CH_1);
    ok &= ledcAttachChannel(A.in2, FREQ, RESBITS, ROBORA_A_CH_2);
    ok &= ledcAttachChannel(B.in1, FREQ, RESBITS, ROBORA_B_CH_1);
    ok &= ledcAttachChannel(B.in2, FREQ, RESBITS, ROBORA_B_CH_2);
#endif

    FirstStart = ok; /*Configurazione pin avvenuta con successo */
    if (ok)
    {
        _coast(A);
        LastTargtA = 0;
        _coast(B);
        LastTargtB = 0;
    }

    return ok;
}

inline void RoBoRa_8833::_writePins(MotorCfg &m, uint32_t d1, uint32_t d2)
{
    ledcWrite(m.in1, d1);
    ledcWrite(m.in2, d2);
}
inline void RoBoRa_8833::_coast(MotorCfg &m)
{
    _writePins(m, 0, 0);
}
inline void RoBoRa_8833::_brake(MotorCfg &m)
{
    _writePins(m, MAX_DUTY, MAX_DUTY);
}
inline void RoBoRa_8833::_applyOutput(MotorCfg &m, int16_t speed, uint32_t duty)
{
    // Calcola i valori del duty cycle in base alla modalità di frenata e alla direzione del motore.
    uint32_t duty1 = 0;
    uint32_t duty2 = 0;

    if (m.inverti)
        speed = -speed; // Inversione comando

    if (FASTSLOWCTRL) // Modalità "Fast" (frenata libera)
    {
        if (speed > 0)
        {
            duty1 = duty;
        }
        else if (speed < 0)
        {
            duty2 = duty;
        }
        else
        {
            _coast(m);
            return; // Esci dalla funzione dopo la frenata
        }
    }
    else // Modalità "Slow" (frenata attiva)
    {
        // La frenata attiva è più efficiente e fornisce maggiore controllo,
        // ma richiede un duty cycle calcolato in modo diverso.
        if (speed > 0)
        {
            duty1 = MAX_DUTY - duty;
            duty2 = MAX_DUTY;
        }
        else if (speed < 0)
        {
            duty1 = MAX_DUTY;
            duty2 = MAX_DUTY - duty;
        }
        else
        {
            _coast(m);
            return; // Esci dalla funzione dopo la frenata
        }
    }

    _writePins(m, duty1, duty2);

    // if (FASTSLOWCTRL)
    //{ /*Fast*/
    //     if (s > 0)
    //         _writePins(m, duty, 0);
    //     else if (s < 0)
    //         _writePins(m, 0, duty);
    //     else
    //         _coast(m);
    // }
    // else
    //{ /*Slow*/
    //     if (s > 0)
    //         _writePins(m, (MAX_DUTY - duty), MAX_DUTY);
    //     else if (s < 0)
    //         _writePins(m, MAX_DUTY, (MAX_DUTY - duty));
    //     else
    //         _coast(m);
    // }
}

// ---- Controllo diretto motori s ∈ [-255..255]----
void RoBoRa_8833::_cmdMotor(MotorCfg &m, int16_t s_raw)
{
    int16_t s = constrain(s_raw, -255, 255); // controlla limiti
    float sf = s / 255.0f;                   // normalizza -1..1
    // deadzone
    float dz = deadzone / 255.0f;
    sf = applyDeadzone(sf, dz);
    // expo
    float e = expoPct / 100.0f;
    sf = applyExpo(sf, e);
    // clamp
    sf = fmaxf(-1.0f, fminf(1.0f, sf));
    // pwm assoluto
    uint32_t dutyTarget = (int)roundf(fabsf(sf) * MaxVel * MAX_DUTY);
    _applyOutput(m, s, dutyTarget);
}
void RoBoRa_8833::setSpeedA(int16_t s) { _cmdMotor(A, s); }
void RoBoRa_8833::setSpeedB(int16_t s) { _cmdMotor(B, s); }
// Frenata / folle
void RoBoRa_8833::brakeA() { _brake(A); }
void RoBoRa_8833::brakeB() { _brake(B); }
void RoBoRa_8833::brakeAll()
{
    brakeA();
    brakeB();
}
void RoBoRa_8833::coastA() { _coast(A); }
void RoBoRa_8833::coastB() { _coast(B); }
void RoBoRa_8833::coastAll()
{
    coastA();
    coastB();
}

void RoBoRa_8833::driveTank(int16_t throttle, int16_t steer, uint16_t max)
{
    // snapshot atomico
    int16_t xi = steer, yi = throttle;

    if (invertiTankthrottle)
        yi = -yi;
    if (invertiTankSteer)
        xi = -xi;

    float Normalize = (float)max;
    // Steer gain
    xi = xi * steerGainPct;

    // normalizza -1..1
    float x = xi / Normalize;
    float y = yi / Normalize;
    // deadzone
    x = applyDeadzone(x, deadzone);
    y = applyDeadzone(y, deadzone);
    // expo
    x = applyExpo(x, expoPct);
    y = applyExpo(y, expoPct);

    // modalità arcade
    // scala la rotazione in funzione della velocità avanti/indietro a fermo: scale=1 (rotazione massima) a piena velocità:
    // scale = 1 - arcadeLvl (con arcadeLvl=0.85 => ~15% della rotazione)
    if (arcadeEn)
        x = x * (1.0f - fminf(1.0f, fabsf(y)) * arcadeLvl);

    // mixing differenziale
    float left = y + x;
    float right = y - x;

    // normalizza se supera 1
    float maxAbs = fmaxf(fabsf(left), fabsf(right));
    if (maxAbs > 1.0f)
    {
        left /= maxAbs;
        right /= maxAbs;
    }
    // clamp
    left = fmaxf(-1.0f, fminf(1.0f, left));
    right = fmaxf(-1.0f, fminf(1.0f, right));

    // pwm assoluto con limite a velocità massima
    LastTargtA = (int)roundf(fabsf(left) * MaxVel * MAX_DUTY);
    LastTargtB = (int)roundf(fabsf(right) * MaxVel * MAX_DUTY);

    /*Applica il comando*/
    if (left >= 0)
        _applyOutput(A, 1, LastTargtA);
    else
        _applyOutput(A, -1, LastTargtA);
    if (right >= 0)
        _applyOutput(B, 1, LastTargtB);
    else
        _applyOutput(B, -1, LastTargtB);
}

inline float RoBoRa_8833::applyDeadzone(float v, float dz)
{
    float av = fabsf(v);
    if (av <= dz)
        return 0.0f;
    return v;
    //// rimappa fuori dalla deadzone a [0..1]
    // float nv = (av - dz) / (1.0f - dz);
    // if (v < 0)
    //     return (-1.0f * nv);
    // else
    //     return (1.0f * nv);
}
inline float RoBoRa_8833::applyExpo(float v, float e)
{
    return (1.0f - e) * v + e * v * v * v;
}

/*Stampa tutta la configuazione */
void RoBoRa_8833::printConfig(void)
{
    Serial.printf("MOTORE A: in1:%d in2:%d ch1:%d ch2:%d inv:%d\n\r", A.in1, A.in2, A.ch1, A.ch2, A.inverti);      // configurazione motore A
    Serial.printf("MOTORE B: in1:%d in2:%d ch1:%d ch2:%d inv:%d\n\r", B.in1, B.in2, B.ch1, B.ch2, B.inverti);      // configurazione motore B
    Serial.printf("CONFIG: ctrl:%d Freq:%d res:%d duty:%d \n\r", FASTSLOWCTRL, FREQ, RESBITS, MAX_DUTY);           // CONFIG PWM
    Serial.printf("CORRETTIVI: kVel:%f Steer:%f dead:%f Expo%f \n\r", MaxVel, steerGainPct, deadzone, expoPct);    // CORRETTIVI
    Serial.printf("ARCADE: Enable:%d level:%f \n\r", arcadeEn, arcadeLvl);                                         // ARCADE
    Serial.printf("TANK: Inversione Throttle:%d Inversione Steer:%d \n\r", invertiTankthrottle, invertiTankSteer); // TANK
}
