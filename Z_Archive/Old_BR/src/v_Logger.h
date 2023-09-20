/*
 * Cette classe gere un logger, qui est un tableau de float, avec chaque case assignée à une valeur
 * périodiquement, RosTask envoie ce tableau sur un topic dédié
 *
 * */
#ifndef __H_LOGGER_INCLUDED
#define __H_LOGGER_INCLUDED

class Logger
{
public :
    enum LogField
    {
        currentTime,
        positionX,
        positionY,
        positionTheta,
        rampePosX,
        rampePosY,
        rampeAngleTheta,
        rampeDistSpeed,
        rampeAngleSpeed,
        commandeMotorL,
        commandeMotorR,
        goalSpeedL,
        goalSpeedR,
        currentSpeedMotorL,
        currentSpeedMotorR,
        erreurAngleAsserv,
        erreurDistAsserv,
        currentStateIndex,
        odoL,
        odoR,
        odoMotorL,
        odoMotorR,
        NbOfFields,
        noField
    };
    static void setFieldValue(float value, LogField fieldName);
    static float* getArrayOfValues();
private :
    static float m_arrayOfValues[NbOfFields];
};
#endif
