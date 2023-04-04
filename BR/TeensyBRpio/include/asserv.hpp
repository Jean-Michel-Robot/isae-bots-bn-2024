#ifndef ASSERV_H
#define ASSERV_H



class asservPID
{

public:

    float k1, k2, k3;

    float KP, KI, KD;

    float target[2];  // vd, omegad

    float errorPos[3];  // ex, ey, etheta
    float cmdV[2];  // v, omega

    void updateError(float errorPos[3]);

    void updateCommand();


private:
    float Rsb[2][2];  // matrice de passage du repère monde vers le repère 
};

#endif  // ASSERV_H
