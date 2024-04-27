#ifndef ASSERV_CPP
#define ASSERV_CPP
#include "asservPID.h"
#ifndef __linux__
#include <Arduino.h>
#endif
#include <cmath>

asservPID::asservPID(float KP, float TI, float TD, float N, float outputMax,float satuIntegrale)
{
    m_KP = KP;
    m_TI = TI;
    m_TD = TD;
    m_outputMax = outputMax;
    m_satuIntegrale = satuIntegrale;
    m_N = N;
}

asservPID::asservPID(float KP, float TI, float TD, float outputMax, float satuIntegrale)
{
    m_KP = KP;
    m_TI = TI;
    m_TD = TD;
    m_outputMax = outputMax;
    m_satuIntegrale = satuIntegrale;
    m_N = 5.0;
}

float asservPID::computeOutput(double error, unsigned long t_micro)
{
    if (m_lastTimeOfCalcul > t_micro || std::isnan(error))
        return constrain(m_KP * error + m_cmdDerivee + m_sumIntegral, -m_outputMax, m_outputMax);	// on renvoie la valeur identique a la derniere iteration
    double deltaT = (double)(t_micro - m_lastTimeOfCalcul) *1e-6;
    m_lastTimeOfCalcul = t_micro;

    if (m_TI != 0.0 && m_enableI) {
        m_sumIntegral += m_KP / m_TI * deltaT * (error+m_lastMesuredError)/2.0;
        m_sumIntegral = constrain(m_sumIntegral, -m_satuIntegrale, m_satuIntegrale);
	} else {
        m_sumIntegral = 0.0;
	}
    if (m_KP != 0.0 && m_TD != 0.0)
        m_cmdDerivee = constrain(m_KP * m_TD * (error - m_lastMesuredError + (m_cmdDerivee / (m_N * m_KP))) / (deltaT + m_TD / m_N) , -m_outputMax, m_outputMax);
	else{
        m_cmdDerivee = 0.;
	}
    float commande = m_KP * error + float (m_sumIntegral) + m_cmdDerivee;
    m_lastMesuredError = error;
    commande = constrain(commande, -m_outputMax, m_outputMax);

	return commande;
}

float asservPID::computeOutputWithDerivateOfError(double error, double derivateOfError, unsigned long t_micro)
{				// meme fonctionnement, mais la derivee de l'erreur est fournie
    if (m_lastTimeOfCalcul > t_micro)
        return constrain(m_KP * error + m_cmdDerivee + m_sumIntegral, -m_outputMax, m_outputMax);	// on renvoie la valeur identique a la derniere iteration
    double deltaT = (double)(t_micro - m_lastTimeOfCalcul) / 1000000.;
    m_lastTimeOfCalcul = t_micro;

    if (m_TI != 0.0 && m_enableI) {
        m_sumIntegral += m_KP / m_TI * deltaT * (error+m_lastMesuredError)/2.0;
        m_sumIntegral = constrain(m_sumIntegral, -m_satuIntegrale, m_satuIntegrale);
	} else {
        m_sumIntegral = 0.0;
	}
    if (m_KP != 0.0){
        m_cmdDerivee = constrain(m_KP * m_TD * derivateOfError, -m_outputMax, m_outputMax);
	}
    float commande = m_KP * error + float (m_sumIntegral) + m_cmdDerivee;
    m_lastMesuredError = error;
    commande = constrain(commande, -m_outputMax, m_outputMax);
	return commande;
}

void asservPID::RAZAtSpecifiedError(float error, unsigned long t_micro)
{
    m_lastMesuredError = error;
    m_sumIntegral = 0.;
    m_lastTimeOfCalcul = t_micro;
    m_cmdDerivee = 0.;
}

void asservPID::RAZ(unsigned long t_micro)
{
    RAZAtSpecifiedError(0.0, t_micro);
}


void asservPID::resetOutput(float error, float output, float error_derivee, unsigned long t_micro)
{				// utile pour les transitions, adapte I pour avoir la continuité de la commande
    m_lastMesuredError = error;
    m_cmdDerivee = constrain(m_KP * m_TD * error_derivee, -m_outputMax, m_outputMax);
    m_sumIntegral = constrain(output - m_KP * error - m_cmdDerivee, -m_satuIntegrale, m_satuIntegrale);
    m_lastTimeOfCalcul = t_micro;
}

void asservPID::setGains(float KP, float TI, float TD)
{
    if(KP == 0.0)
        return; // on rejette les gains non valides
    m_KP = KP;
    m_TI = TI;
    m_TD = TD;
}

void asservPID::setI(bool enable)
{
    if ((enable && !m_enableI) || (!enable && m_enableI))	// si on modifie l'etat de l'integrale on la vide
        m_sumIntegral = 0.0;
    m_enableI = enable;
}

void asservPID::setI_satu(double isatu)
{
    m_satuIntegrale = isatu;
    m_sumIntegral = constrain(m_sumIntegral, -m_satuIntegrale, m_satuIntegrale);
}


double asservPID::getIntegrale() const
{
    return m_sumIntegral;
}

double asservPID::getDerivee() const
{
    return m_cmdDerivee;
}

float asservPID::getLastCmd() const
{
    return constrain(m_KP * m_lastMesuredError + m_cmdDerivee + m_sumIntegral, -m_outputMax, m_outputMax);	// on renvoie la valeur identique a la derniere iteration
}

float asservPID::getKP() const
{
    return m_KP;
}

bool asservPID::areGainsOk() const
{
    return m_KP != 0.0 && m_satuIntegrale > 0.0;
}
#endif
