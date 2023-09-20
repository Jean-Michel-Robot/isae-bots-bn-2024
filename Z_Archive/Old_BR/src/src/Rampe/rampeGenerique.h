#ifndef __H_RAMPE_GENERIQUE
#define __H_RAMPE_GENERIQUE

#ifdef __linux__
#ifdef __SIMU__
#include "../../../Simulation/Arduino_defines.h"
#else
#include <string>
// include a rajouter pour compiler en C++ sur un PC
#ifndef min
#define min(A,B) ((A)>(B)? (B): (A))
#endif
#ifndef sq
#define sq(A) ((A)*(A))
#endif
#endif
#else
#include <Arduino.h>
#endif
#include <cmath>
class RampeGenerique
{
public :
    RampeGenerique(float acceleration, float decelerationSpeedShift, float decelerationArret, float speedGoal);
    RampeGenerique (float accelerationHard, float deccelerationSpeedShiftHard,float deccelerationArretHard,float accelerationSmooth, float deccelerationSpeedShiftSmooth, float deccelerationArretSmooth, float goalSpeed);
    void setAccDecc(float accelerationHard, float deccelerationSpeedShiftHard,float deccelerationArretHard,float accelerationSmooth, float deccelerationSpeedShiftSmooth, float deccelerationArretSmooth);
    void setAccDecc(float acceleration, float deccelerationSpeedShift, float deccelerationArret);
    void setDeccBrake(float deccelerationBrake){m_deccelerationBrake = deccelerationBrake;}
    void setSpeed(float speed,float time); // on defini la vitesse max a obtenir, on peut la modifier a la volée
    void startParcours(float time); // on lance la rampe

    float calcSpeed(float time) const; // calcul de la vitesse actuelle (commun angle/distance)
    bool isFinished(float time)const; // teste si la rampe est en cours ou pas
    bool isWorking(float time)const; // retourne false si il y a une erreur interne a la rampe
#ifdef ARDUINO
    String getStringError(float time) const; // donne une string qui décrit le type d'erreur interne
#else
    std::string getStringError(float time) const; // donne une string qui décrit le type d'erreur interne
#endif

    float getAcc() const {return m_acceleration;}
    float getDeccArret() const {return m_deccelerationStop;}
    float getTimeEnd()const{return m_timeEnd;} // donne le temps où la rampe s'arrete
    float calcDistChgmtVitesse(float speedStart, float speedEnd) const;// calcule la distance necessaire pour effectuer la variation de vitesse donnee

    static float s_calcDistChgmtVitesse(float speedStart, float speedEnd, float acceleration, float deceleration);
protected: // on restreint l'acces a certaines methodes a uniquement les classes filles, pour éviter toute confusion
    enum DynamicType
    {
        FAST,
        SMOOTH,
        BRAKE
    };
    float _calcLinPos(float time) const;
    void _setLinParcours(float posLinStart, float posLinEnd,DynamicType dynamicType, float startSpeed, float endSpeed);
    enum Etat_Rampe
    {
        ETAT_WAITING_PARCOURS = 2, // attend le parcours a effectuer
        ETAT_WAITING_START = 3, // attend le départ
        ETAT_RUNNING = 4 // a demarré
    };

    enum Cas_Rampe
    {
        CAS_COMPLET = 1, // rampe en /-\ : acceleration-vitesse constante - deceleration
        CAS_GOAL_SPEED_NON_ATTEINT = 3, // rampe en /\ acceleration puis deceleration sans atteindre la vitesse objectif
        CAS_DOUBLE_DECELERATION = 2 // rampe en \-\ deceleration puis vitesse constante puis deceleration : se produit lorsqu'on diminue à la volée la vitesse
    };
    //booleens de l'etat de la rampe : le set a ete fait, le parcours a ete demarre, le parcours est en mode distance ou non
    //	       	bool parcoursNotSet,parcoursNotStarted;
    Etat_Rampe m_etat = ETAT_WAITING_PARCOURS;

    float m_goalSpeed =0.0; // vitesse objectif
    float m_posLinFin,m_posLinStart; // position lineaire, c'est à dire projetee sur le parcours

    float m_startTime = 0.; // temps de début de la rampe
    float m_timeFinAccelerationInitiale = 0.; // fin de l'acceleration initiale ou de la 1ere deceleration le cas échéant
    float m_timeDebutFreinage = 0.; // debut du freinage
    float m_timeEnd = 0.; // fin de la rampe

    float m_distFinAccelerationInitiale= 0.; // distance parcourue a la fin de l'acceleration initiale
    float m_distDebutFreinage=0.; // distance parcourue avant de decelerer
    float m_distEnd = 0.;
    float m_startSpeed = 0.,m_vitesseEtablie = 0.,m_endSpeed = 0.; // vitesse de demarrage,vitesse max atteinte et vitesse à l'arrivée
    Cas_Rampe m_cas; /* cas actuel : 1 = /-\, 2 =\-\ ,3 = /\  */

    static bool s_isValid(float val); // informe si la variable est valide
    void _calcTimesDist(float distLeft,float currentTime,float currentSpeed,float endSpeed); // cette fonction calcule les temps et distances des changements de phase

    // ces accelerations peuvent etre modifiées à tous moment et ne sont pris en compte qu'à la création de la rampe
    float m_accelerationHard; // mm.s-2 // accelération au démarrage
    float m_deccelerationSpeedShiftHard; // mm.s-2 // decélération pour ajuster la vitesse en cas de changement
    float m_deccelerationStopHard; //mm.s-2 // décélération pour l'arrêt à l'arrivé
    float m_accelerationSmooth; //mm.s-2 // idem mais en rampe de type smooth (isRapid = false)
    float m_deccelerationSpeedShiftSmooth; //mm.s-2
    float m_deccelerationStopSmooth; //mm.s-2

    float m_deccelerationBrake;
private :
    // ces accelerations sont utilises dans tous les calculs et uniquement modifiés à la création d'un nouveau parcours (sauf changement forcé)
    float m_acceleration ;//Acceleration au départ et en cas d'augmentation de la consigne
    float m_deccelerationSpeedShift; // deceleration maximale (en cas de changement de consigne)
    float m_deccelerationStop;//deceleration douce pour l'arret
};

#endif

/*
oooooooooooooooooooooooooooooooooossoosssyhddddhsosyyyyssyysssoooooooooooooooooooooooooooooooooooooo
oooooooooooooooooooooooooooooooooyddhddhdNmdmNmdhhhhyoosddhddhhhysoooooooooooooooooooooooooooooooooo
oooooooooooooooooooooooooooooosydmNNNmhmNNdhmmddhhdyssydmmdddmddyoo++++ooooooooooooooooooooooooooooo
oooooooooooooooooooooooooosyddmmmNNNNmmNNdhhmmmhsssyhhsyhmmdddmmdso+/+oooooooooooooooooooooooooooooo
ooooooooooooooooooooooooydmmNmmdmNNNNmmNmhdmNmyosshhhs+sdhddmmhdmdo//+//+ooooooooooooooooooooooooooo
ooooooooooooooooooooooydmNNNmmmNNNNNNNmNddmmdhhyhyydh+ysoyyyhmhhhhso+///::+ossoooooooooooooooooooooo
ooooooooooooooooooossdmmmmmNNNNNNNNNNNNNNNNmmmmmmhdhyyhohh/oydhyyhy+o+//////osoooooooooooooooooooooo
oooooooooooooooooosydmmmmNNNNNNNNNNNNNNNNNNmNNNmmddddhydds+osyydyssyho::::::/+oooooooooooooooooooooo
ooooooooooooooooyhddddmmNNNNNNNNNNNNNNNNNmmNNmmddhdddddhhhsyhohdyoosys+--:://+ssoooooooooooooooooooo
ooooooooooooooshddddmmNNNNNNNNNNNNNNNNmmddmmmdddhhddmdhhhhhhyyhyoo+////:...-/+yysooooooooooooooooooo
ooooooooooooosydmmNNNNNNNNNNNNNNNNNNNNmdddmmdhddmmdhyyhhhhddhyyo++:::.--.```.:+o+ooooooooooooooooooo
ooooooooooooohdmNNNNNNmNNNNNNNNNNNNNNNdddhhhhyyyyyssssssysyyssso++:::.``.`   `-///+oo++ooooooooooooo
oooooooooooohmmmNNmmmmmmmmmNNNmmmddddmmmddhhhhssoo+++++++osssssoo+/:-.` `      .+/:oo+oooooooooooooo
oooooooooosdmmmNmdhyyhhdmmmmmdhhhhyyyhddmmmdhyssoo++++++oooooooo+/::-.`         .+/:+sssoooooooooooo
ooooooooosdmmNNmdysssshhhhhyyyyyyyyyyyyyyyyyssssooo+++++++++++++/:::-..          -//:+ooo+oooooooooo
ooooooooohdmmNmmysssshhssssssssssssssssssssssssssoo+++++++///////::-..`           ::::osoooooooooooo
ooooooooodmmmmdysooosysssssssssssssssssssssssssssooo+++///////:::::-..`           `--:/soooooooooooo
oooooooohmmmdhyssoooossssssssssssssssssssssssssssooo+++////::::::--...`            .--://ooooooooooo
oooooooymmddhysoooooooossssssssssssssssssssssssssooo++++///::::::--..`              ---:-+oooooooooo
ooooooohdmddhssoooooooooossssssssssssssssssssssssssoooo+++////::---..``             .-::.-/o+ooooooo
ooooooodmmddysoooooooooooosssssssssssssssssssssssssssoooo+++///::--..``             `:::--:o++oooooo
ooooooodmmdhysoooooooooooosoosssssssssssssssssssssssssooo+o+++//:--..``              :/:::://ooooooo
ooooooohmmddysoooooooooooossssssssssssssssssssssssssssooooooo+++/:--...``            -//:://:ooooooo
ooooooosmmmdyssoooooooosssssssssssyyysyyysssssssssssssoooooooo++//::---..``          -/////:/oyooooo
oooooooodmmdhssooooossssssyyyyyyyyyyyyyyyysssssyyssssssooooossoo++///:::---.`        -:/+/:::/sooooo
oooooooohmmdhssssoosyyyyhhhhhhddddhhhyhyyyyyyyyyyyyssssssssssssysssosoooo//:-.`      -/+///::ssoosoo
oooooooodmmmdyssssyhhddddmmmmNmmmmmmddhhhhyyyyyyyyyysssssyyyyhhhhdhhhdhhhyo//:-.`    :++///+ohdsssoo
ooooooosdmmNmhsyhddmmmmmmNNNNNmmmmmmmdddhhhhhyyyyyyysssssssyyhhdddmddddddhhyso+/:-.  /+//ooossysoooo
ooooooosddhmmhyhmmmmmmdddddddddmmmmddddhhhhhhyyyyyysooooooshhhhhyyyyyyysssssooosso+:-+++oo//+oysoooo
ooooooosdddmNNmmdhhhhhhhhhhhhhhhhhhddmdhhhhhhhhhyyso+++oyhhyyssssyyyyyssssoo+//:/sdddhs++:/://ysoooo
oooooooyhNNNNNmddhhhhhhhhdddddddddhhhddmmmmmmmmmmmdddhhdsoosyysyyyhhhhhys+/+o+/::-+ddds++++/++ysoooo
oooooooosdNNNdyhhyyyhhhdddmmNNNmddddhhhhmNmhhhhyso+//dm+/+osyyhhyshhdmdh++++/+/:-`.yds//::+++ssooooo
ooooooooosdNNhsyhysyhhhdhhdmmNmmdddddhhyyNdhhhyso+:.`do-:/+syyhyyyyhddhs//:---..` .sh.:/:-+//soooooo
oooooososyddNhsyhysssssyyhhdddddddhhhhhyhmdhhyyso+/::o+--::/osyyyssssso+/:-``     .o: -::-/-shoooooo
ooooooooooyydhssyyosssssyyyyyhhhhhhhhyyhdddhyysssssoo:+:---::/+osssooo+//:-.`     :`  .--.--ssoooooo
oooooooso+oyhhsossossssssyyyyyyyyyyyyyyhydhyyyssso++o+o..--...-:/+++++//::-.`    -.   .:-../oooooooo
oooooooooo+shhsssssssssssyyyyyyyyyyyyyyyhhyyyysso++//+o+.```````.-::://:::-.`  `-.    .--..ooooooooo
oosooooooooshdysssssssssyyyyyyyyyyyyyyyyyyyyyssoo+//:::::-````````.------..``.--`     .-...ooooooooo
sooooooooooyddysossyssssyyyyyyyyyhhhhhhyyyyssoo++/::-....:/-`.---..--......--.`       .--.-ooooooooo
sooooooooooohdysoosssyyyyyyyyyyyyhhhhyyyysssssoo+/:-`````.:/-..:++/////::--`          ...`/ooooooooo
soooooooooooyhhssossssssyyyyyyyhhhhyyyysssssysso+/:-.`.``.-:/-``oso+/::--.``         `-..`/ooooooooo
ooooooooooooshdsoosssssssysyyyyyhhysssssssssyysso+::---.`.---.`-sssso+/::-..``       .-...+ossoooooo
oooooooooooo+ohyoosssssssssyyyyyyhyysyyyyyyyyyyyso////:--.---.-ossssoo+//:-.``       .--:+ooosoooooo
ooooooooooooo/oyoosssssssssyyyyyyyyyyyyhhhhhyyyysoo+++++///::/+++++oooo++/:-.``      .:+oooooooooooo
oooooooooo+o+oooooossssssssyyyyyyyyyyyyyyhhhhhyyyyssyyyyso+//::::///++++++/--.``     .:+oooooooooooo
oooooooooooo+oo+soossssssssyyyyyysssyyyyyyhhhyyyyyyyyyyysooo//:--::://++++/:-.`      .:ooooooooooooo
oooooooooooo+oooossoosssssyyhyyyyyyyyyyhhhhyyyyyyyyyyysyysoo++::-::://++o+/:-.`      -+ooooooooooooo
oooooooooooo+ooooossssssssyhhhhhhhyyhhyyyyyyyyyyyyyssysssssoo+/////+oooooo+:-.`     `/oooooooooooooo
oosooooooooooooooosssoosssyhhhhhhhhddhhyyyyyyyyyyyyyyysssssoo++ossosssysso/:-.``    .+oooooooooooooo
oosooooooooooooooosssssssssyyhyyyyyyhhhhhhhhhhhhhhhhyysso+/::--:///+osssso/:-.``````.+oooooooooooooo
ooooooyooooooooooo+ssssssssyyyyyysssssyyyyyyyyyyyyysso+//:-.````..-:/+osso/-.....``` `:oooossosyssoo
oooooooooooooooooo+osssssssyyyyyysssssssssssssssyyssso+//:-.``````.-:/+ooo/-.....`.`   `/oososoooooo
oooooooooooooooooooossssssssyyyyssssssssssssssssssssssoo+/:-.`````..-:/+++:..-....-.`    `.:/ooooooy
ooooooooooooooooooosyyyssssssysssssssssyyyssyyyyyysssso++/::-..```...-::/:-------:--`        `.-:::/
ooooosooooooooooosyyyhhyyssssysssssssssyyyyyyyyyyyyssssoo+/:-..```...---------:/+/---`        ``````
oooooooooooo+oossyyyhhddyyyssyyysssssssssyyyyyyyyssssoo+//:-...`````..------:/oss+:--.`        `````
oosssssssso+oosyyhhhhdddssyyyyyysssssssssssyysssssoo+++/::--..```` ``..-::-/ssyyys::--.         `...
ssyyyyyyo++oossyhhhhhdmdssssyyyyyyssssssssssyysso++++//:--....``````.-::-.:hddddhh/::--`        .---
syyyhyhs++oossyhhhhhdmmmssoosyyyyyysssssssssssssooo+++/:-......```.-::-``.oyhhddddo:::-.        .:::
yhdddds++oossyyhddddmmNNssoooossyyyyyssssssssssssosoo++/:--.....`-:-.```./yddhsssso:::-.`        .-:
dhhyso+++oossyyhdddmNNNdsssoooooosssyyysssssssssssssooo+//:-----:-.```..:shhdmdsss+:::-.`          `
so++++++ooosyyyhhmmNNmmhssooooooooosssyyysssssssyyssssooo+/:::--......-:oyhhhdddss//:::.``         `
o+++++++ooosyyyhdNNNmmdhyssoooooooooooosssyssyyssssssssoo+/:--......--:+yhhhhhhhys//::-.`          .
o+++++++ooosyyyhmNNmmddhyssoooooooooooooooossssssssoo+//:-----------:/+yhhhhhhddh+///:-.`         `-
o++++++++oossyyydNmmmdhhyyssoooooooooooooooooossooo+++//:::--::---::/+shhhdhysydh///::-.`        `:/
*/
