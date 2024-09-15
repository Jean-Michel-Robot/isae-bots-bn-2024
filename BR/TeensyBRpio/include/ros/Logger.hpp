/*
 * Cette classe gere un logger, qui est un tableau de float, avec chaque case assignée à une valeur
 * périodiquement, RosTask envoie ce tableau sur un topic dédié
 *
 * */

#ifndef H_LOGGER
#define H_LOGGER

class Logger
{
public:
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

        goalPointSpeedX,
        goalPointSpeedY,

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
    static float *getArrayOfValues();
    static void loop();

private:
    static long loop_timer;
    static float m_arrayOfValues[NbOfFields];
};

#endif
