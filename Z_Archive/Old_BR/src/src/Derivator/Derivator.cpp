#ifndef __DERIVATOR_CPP_INCLUDED
#define __DERIVATOR_CPP_INCLUDED
#ifndef __linux__
#include <Arduino.h>
// en embarqué
#elif defined(__SIMU__)
// pour la simu
#include "../../../Simulation/Arduino_defines.h"
#else
// pour tests en détaché
#define constrain(A,B,C) ((A)<(B)?B : (A)>(C)? C:A)
#endif
#include "Derivator.h"
#include <cmath>
Derivator_2::Derivator_2(float omega, float Q, unsigned long t_micros,float satu)
{
    m_a3 = -1;
    m_a1 = omega * omega;
    m_a2 = -1 / (Q * omega);
    m_y = 0;
    m_yd = 0;
    m_yd2 = 0;
	m_lastCalc = t_micros;
    m_satu = satu;
}

void Derivator_2::update(float input, unsigned long t_micros)
{
	float dT = float (t_micros - m_lastCalc) * 1.e-6;
	m_lastCalc = t_micros;
	// on met a jour les etats (cf simulink) de gauche a droite
    float old_yd2 = m_yd2;
    float old_yd = m_yd;
    m_yd2 = m_a1 * (input + m_a3 * m_y + m_a2 * m_yd);  //1er bloc
    m_yd += (m_yd2 + old_yd2) * 0.5 * dT;	// on integre selon la methode des trapezes
    m_y += (m_yd + old_yd) * 0.5 * dT;  //dernier bloc
}

float Derivator_2::getOutput() const
{
    return constrain(m_y, -m_satu, m_satu);
}

float Derivator_2::getDerivate() const
{
    return constrain(m_yd, -m_satu, m_satu);
}

float Derivator_2::getDoubleDerivate() const
{
    return constrain(m_yd2, -m_satu, m_satu);
}

void Derivator_2::RAZ(unsigned long t_micros)
{
    m_y = 0;
    m_yd = 0;
    m_yd2 = 0;
	m_lastCalc = t_micros;
}

#endif
