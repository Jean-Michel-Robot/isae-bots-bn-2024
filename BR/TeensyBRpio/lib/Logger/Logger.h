/*
 * Cette classe gere un logger, qui est un tableau de float, avec chaque case assignée à une valeur
 * périodiquement, RosTask envoie ce tableau sur un topic dédié
 *
 * */

#ifndef H_LOGGER
#define H_LOGGER

class Logger
{
public :
    // enum LogField
    // {
    //     currentTime,
    //     positionX,
    //     positionY,
    //     positionTheta,
    //     rampePosX,
    //     rampePosY,
    //     rampeAngleTheta,
    //     rampeDistSpeed,
    //     rampeAngleSpeed,
    //     commandeMotorL,
    //     commandeMotorR,
    //     goalSpeedL,
    //     goalSpeedR,
    //     currentSpeedMotorL,
    //     currentSpeedMotorR,
    //     erreurAngleAsserv,
    //     erreurDistAsserv,
    //     currentStateIndex,
    //     odoL,
    //     odoR,
    //     NbOfFields,
    //     noField
    // };

    enum LogField
    {
        currentTime,

        robotPosX,
        robotPosY,
        robotPosTheta,

        goalPointPosX,
        goalPointPosY,
        goalPointPosTheta,

        trajectoryS,
        goalSpeedLinear,
        goalSpeedAngular,

        asservErrorX,
        asservErrorY,

        commandV,
        commandOmega,

        commandeMotorR,
        commandeMotorL,

        rampSpeed,
        
        rampState,
        BrState,

        NbOfFields,
        noField
    };



    static void setFieldValue(float value, LogField fieldName);
    static float* getArrayOfValues();


private :
    static float m_arrayOfValues[NbOfFields];
};

#endif
