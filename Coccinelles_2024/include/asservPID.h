#ifndef ASSERV_PID_H
#define ASSERV_PID_H
class asservPID
{
    /* gère un bloc PID
     * posibilité d'injecter directement la dérivée de l'erreur
     * possibilité de couper momentanément l'effet intégral
     * I capé par une saturation
     * D filtré selon un facteur N ->https://fr.wikipedia.org/wiki/R%C3%A9gulateur_PID
     * */
    /**
     * Cette Classe est générale et permet d'implémenter un bloc PID sur n'importe quel système. Pour l'implémentation sur le robot
     * voir ./src/Asserv.cpp
     */
public:
    asservPID(float KP, float TI, float TD, float m_N, float outputMax, float satuIntegrale); // constructeur par defaut
    asservPID(float KP, float TI, float TD, float outputMax, float satuIntegrale);            // constructeur, avec N = 10

    float computeOutput(double error, unsigned long t_micro);                                        // commande de base
    float computeOutputWithDerivateOfError(double error, double deriv_error, unsigned long t_micro); // commande de base, avec la derivée de l'erreur donnée directement

    void RAZAtSpecifiedError(float error, unsigned long t_micro);                            // on remet a zero le PID en s'adaptant a l'erreur donnée
    void RAZ(unsigned long t_micro);                                                         // on remet le PID a zero
    void resetOutput(float error, float output, float error_derivee, unsigned long t_micro); // utile pour les transitions, adapte I pour avoir la continuité de la commande

    void setGains(float KP, float TI, float TD); // on adapte les gains
    void setI(bool enable);                      // active ou desactive l'integrale
    void setI_satu(double isatu);                // on modifie la saturation sur l'integrale

    double getIntegrale() const; // donne la valeur de l'effet integral sur la commande
    double getDerivee() const;   // donne la valeur de la derivee sur la commande
    float getLastCmd() const;    // donne la derniere valeur calculée pour la commande
    float getKP() const;
    bool areGainsOk() const;

private:
    double m_sumIntegral = 0.0;
    double m_lastMesuredError = 0.0;
    unsigned long m_lastTimeOfCalcul = 0;
    float m_outputMax = 0.0;     // saturation de la commande = max possible renvoyé par computeOutput
    float m_satuIntegrale = 0.0; // Saturation de l'integrale , je vous conseille une valeur élévé au début (ex 5.0)
    float m_cmdDerivee = 0.0;

    bool m_enableI = true;
    float m_N = 5.0;
    float m_KP = 0.0, m_TI = 0.0, m_TD = 0.0;
};
#endif
