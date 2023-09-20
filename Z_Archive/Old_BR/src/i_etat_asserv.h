    /*
 * MAchine a etat, coeur un peu bordélique du code
 *
 * plusieurs principales fonctions sont utiles depuis l'extérieur
 *  -> departRobot qui démarre le robot, on le remet en mode orientation si necessaire
 *  -> updateStateAndComputeAsservObjective qui calcule le prochain état, puis calcule l'objectif de l'asservissement en cours
 * il y a d'autres fonctions
 */
#ifndef __H_ETAT_ASSERV
#define __H_ETAT_ASSERV
#include "a_geometric_tools.h"
#include "a_Led_RGB.h"
#include "a_define.h"
#include "k_Asserv.h"
#include "u_ROS.h"
#include "src/asservPID/asservPID.h"
#include "src/Position2D/Position2D.h"
#include "src/FilterLowPass/FilterLowPass.h"
#include "src/Timer/Timer.h"
#include "src/Rampe/rampes.h"

class MachineAEtatAsserv
{
public :
    MachineAEtatAsserv();

    enum GoalType { // type d'objectif recu par le haut niveau
      FINAL = 0,             // point final, avec orientation
      TRANS = 1,             // point transitoire, sans orientation finale
      STOP  = 2,             // arret "d'urgence"
      RESET = 3,             // reset de la position odometrique
      CONTROL = 4,           // controle par manette
      RECALAGE_AVANT = 5,    // recalage en marche avant
      RECALAGE_ARRIERE = 6,  // recalage en marche arriere
      LIGHT_FINAL_AVANT = 7,  // point final marche avant (smooth)
      LIGHT_FINAL_ARRIERE = 8,// point final marche arriere (smooth)
      ORIENT = 9,             //orientation seule sur place
      FAKE_RECAL_AVANT = 10, // faux recalage en marche avant (plaquage au mur uniquement)
      FAKE_RECAL_ARRIERE = 11,
      CONTROL_SPEED = 12, // controle de l'asserv moteur en direct
      UNVALID_GOALTYPE = 13, // sert a rejeter les valeurs non conformes
    };

    struct OrderCaracteristics // caracteristiques de l'ordre en cours d'execution
    {
        Position2D goalPos;
        GoalType  goalType = STOP;
        // additive field computed from goalPosId
        bool isRecalApplied = true;
    };

    enum EtatAsserv {
        ETAT_STOP = 0, // le robot est arreté, moteurs bloqués
        ETAT_STOPPING = 1, // le robot freine au max
        ETAT_ORIENTATION_INITIALE = 2, // on s'oriente vers la position objectif
        ETAT_AVANCE_RAMPE = 3, // on avance selon la rampe de facon rectiligne
        ETAT_ORIENTATION_FINALE = 4, // on s'oriente vers le cap objectif
        ETAT_ARRIVE = 5, // on est arrive, on reste asservi
        ETAT_RECALAGE = 6, // on se recale
        ETAT_RECALAGE_TERMINE = 7, // le recalage est fini
        ETAT_BLOQUE = 8, // on bloque les moteurs avec l'asserv moteur
        ETAT_ERREUR = -1 //erreur dans l'asserv, on bloque les moteurs
    };
    AsservPositionTask::AsservObjectif updateStateAndComputeAsservObjective(); // effectue les transitions d'etat puis donne l'objectif d'asserv actuel
    void setTypeOfTrajectory(bool isFast);

    bool isAsservWorking() const;
    bool isWheelAllowedToDrift() const; // informe si l'etat actuel autorise le patinage
    bool isRobotArrived() const;

    void departRobot(OrderCaracteristics order);
    void progressiveStop(OrderCaracteristics order); // on demande un stop progressif
    void immediateStop(OrderCaracteristics order){m_goalOrder = order ;_transitionEtat(ETAT_STOP);}
    void error(String details);

    void manageNewOrder(Position2D const& posOrder, GoalType type);
    OrderCaracteristics getGoalOrder() const;

    RampePosition* getRampePosition(){return &m_rampePosition;}
    RampeOrientation* getRampeOrientation(){return &m_rampeOrientation;}
    static String s_orderToString(MachineAEtatAsserv::GoalType obj);
private :
    void _transitionEtat(EtatAsserv next_etat);
    bool _calcTransitionEtat();
    bool _manageTimeout();
    bool _testObjectivePointTooFar(AsservPositionTask::AsservObjectif objectiveToReturn);
    bool _testRampeStatus();
    bool _testSwitchCoherentWithRecalage();
    void _updateLed(MachineAEtatAsserv::EtatAsserv etat);

    Position2D _getPointProjectedInRobotReferential(Position2D const& pos) const;
    float _getAngleVersObjectifIncludingMarcheArriere() const; // calcule l'angle que dois prendre le robot pour se diriger vers le point objectif, en incluant le +PI de la marche arriere
    float _getErreurAngleVersObjectifIncludingMarcheArriere() const; // calcule l'angle que dois prendre le robot pour se diriger vers le point objectif, en incluant le +PI de la marche arriere
    float _getThetaObjectiveInDistRampeAfterTransition()const; // on calcule l'angle objectif de l'asserv angle pendant les rampes en distance
    bool _isCurrentOrderAskRampeDistInReverse() const{return m_goalOrder.goalType == LIGHT_FINAL_ARRIERE;}
    bool _isCurrentOrderAskRecalage() const {return m_goalOrder.goalType == RECALAGE_AVANT || m_goalOrder.goalType == RECALAGE_ARRIERE || m_goalOrder.goalType == FAKE_RECAL_AVANT || m_goalOrder.goalType == FAKE_RECAL_ARRIERE ;}
    bool _isOrderNeedInitialAndFinalRotation() const{return m_goalOrder.goalType == FINAL || m_goalOrder.goalType == LIGHT_FINAL_AVANT || m_goalOrder.goalType == LIGHT_FINAL_ARRIERE;}
    bool _isOrderAskSmoothness()const {return m_goalOrder.goalType == LIGHT_FINAL_AVANT || m_goalOrder.goalType == LIGHT_FINAL_ARRIERE ;}

    //seuils pour les transitions d'etat
    static constexpr float SEUIL_DIST_LATERAL = 20;  // mm // erreur lateral pour le passage rampe->orientation finale
    static constexpr float SEUIL_DIST_FRONTAL = 5; // mm // idem en avance
    static constexpr float SEUIL_ORIENT = degToRad(4.5);    // rad //erreur d'orientation pour la passage orientation initale->rampe
    //seuils pour l'arrivée
    static constexpr float SEUIL_DIST_ARRIVE = 5;      //mm // seuils pour declencher le okTurn final
    static constexpr float SEUIL_ANGLE_ARRIVE = degToRad(3.6);   //rad
    //seuils recalage
    static constexpr float SEUIL_BLOCAGE_MUR = 20; //15 //mm.s-1 // seuil de "vitesse" sur les odometres positions pour considerer le robot au contact
    static constexpr float DIST_TRANSITION_ANGLE = 10; //mm // distance pendant laquelle on fait la transition de consigne d'angle // au début de la rampe en position, on garde la direction de la fin de la rampe de rotation, et on transitionne vers thetaObj
    static constexpr float SEUIL_ERREUR_DIST_RECAL = 300; //mm // distance max avec le point objectif pour se recaler
    static constexpr float SEUIL_ERREUR_ANGLE_RECAL = degToRad(12.0); //rad //angle d'ecart max avec la consigne d'angle
    //coef pour se diriger
    static constexpr float COEF_ERREUR_LATERALE = 0.006; // rad/mm // coef de prise en compte de l'erreur laterale dans l'angle consigne pendant la rampe position

    //temps de timeout/timemin/recalages
    static constexpr float TIMEMAX_AVANCE = 3 ;//s // temps max ou le robot cherche à converger à l'arrivée
    static constexpr float TIMEMIN_AVANCE = 0.1; //s // temps min ou le robot cherche à converger
    static constexpr float TIMEMAX_ORIENT = 3; // s // temps max ou le robot cherche à converger à son orientation
    static constexpr float TIMEMIN_ORIENT = 0.1; //s // temps min ou le robot cherche a converger
    static constexpr float TIMEOUT_FREINAGE = 2; //s //temps max de freinage avant un blocage total des roues
    static constexpr float TIMEMAX_RECAL_AVANCE = 4; //s //temps maximum du déplacement d'une phase de recalage
    static constexpr float TIMEMAX_RECAL_STOP_WHEEL = 0.5; //s // temps maximum où on laisse les roues s'arrêter après avoir patiné contre le mur
    static constexpr float TIME_CONTACT_RECAL = 0.1; //s //temps pendant lequel le robot reste au contact du mur en patinant
    static constexpr float TIMEOUT_DIST_TOO_GREAT = 0.05; //s // temps au dela duquel le robot se déclare hors trajectoire
    static constexpr float TIME_ASSERV_RECAL = 1; //0.5; // s // temps ou l'asserv angle est active pendant les phases de recalage
    static constexpr float TIMEOUT_SPEED_TOO_HIGH_WHILE_ARRIVED = 0.2; // s // temps pdt lequel on autorise une vitesse elevee sur les moteurs alors qu'on est arrives

    FilterLowPass m_filterSpeedRecalL = FilterLowPass(TIME_CONTACT_RECAL / 4); // filtres de vitesse des roues pendant le placage au mur (recalage)
    FilterLowPass m_filterSpeedRecalR = FilterLowPass(TIME_CONTACT_RECAL / 4);
   //timers
    Timer m_timerBufferTimeOrientation = Timer(TIMEMIN_ORIENT);      // timemin orientation
    Timer m_timerTimeOutOrientation = Timer(TIMEMAX_ORIENT);         // timeout orientation
    Timer m_timerBufferTimeAvance = Timer(TIMEMIN_AVANCE);           // timemin phase d'avance
    Timer m_timerTimeOutAvance = Timer(TIMEMAX_AVANCE);              // timeout phase d'avance
    Timer m_timerRecalage = Timer(TIME_CONTACT_RECAL);                   // timer ou le robot patine contre le mur
    Timer m_timerOutRecalageAvance = Timer(TIMEMAX_RECAL_AVANCE);                // timeout recalage
    Timer m_timerOutRecalageStopWheel = Timer(TIMEMAX_RECAL_STOP_WHEEL);                // timeout recalage
    Timer m_timerOutFreinage = Timer(TIMEOUT_FREINAGE);                // timeout freinage, au delà on bloque les roues
    Timer m_timerTimeOutPositionErrorTooGreat = Timer(TIMEOUT_DIST_TOO_GREAT);//timeout écart à la position théorique élevé pendant trop longtemps
    Timer m_timerRecalageAsservOn = Timer(TIME_ASSERV_RECAL);           // temps pendant lequel l'asserv en angle est active
    Timer m_timerOutSpeedTooHighWhileArrived = Timer(TIMEOUT_SPEED_TOO_HIGH_WHILE_ARRIVED); // cf au dessus

    Position2D m_startPosBeforeRotation;//Position de départ de la rampe (avant la rotation)
    Position2D m_startPosAfterRotation;//Position où est le robot quand il démarre la rampe en Position
    Position2D m_startPosRecal;// positions ou on commence a avancer pour se plaquer au mur (recalage)

//etat actuel de l'asserv et ordre en cours
    OrderCaracteristics m_goalOrder;//Position objectif envoyé par ROS
    OrderCaracteristics m_lastGoalOrder;// derniere position Objectif, utilisée pour assurer la continuité de la commande

    static constexpr AsservPositionTask::AsservObjectif OBJECTIFSTOP{AsservPositionTask::OBJECTIF_COMMANDE,0.0,0.0,0.0,0.0,0,0};

    EtatAsserv m_currentStateAsserv = ETAT_STOP; // par défaut on démarre en stop
    // File d'attente des états asserv complétée par ros
    //std::queue<EtatAsserv> fileOfSTates; // do not forget #include <queue>

//Rampes
    RampePosition m_rampePosition = RampePosition(DEFAULT_ACC_DIST,DEFAULT_ACC_DIST,DEFAULT_ACC_DIST,DEFAULT_SPEED_DIST);
    RampeOrientation m_rampeOrientation = RampeOrientation(DEFAULT_ACC_ANGLE,DEFAULT_ACC_ANGLE,DEFAULT_ACC_ANGLE,DEFAULT_SPEED_ANGLE);
    //accelerations par defaut
    //Valeur de la PMI as of 26 mai 2022
    static constexpr float DEFAULT_ACC_DIST = 500; // mm.s-2
    static constexpr float DEFAULT_SPEED_DIST = 1000; //mm.s-1
    static constexpr float BRAKE_DECCELERATION = 500; // mm.s-2

    static constexpr float DEFAULT_ACC_ANGLE = 5; // rad.s-2 // WTF, non mis à jour
    static constexpr float DEFAULT_SPEED_ANGLE = 10.0 ; //rad.s-1

    RampeVitesseOnly m_rampeRecal = RampeVitesseOnly(ACC_RECAL,SPEED_RECAL);
    static constexpr float SPEED_RECAL = 35; // mm.s-1 // TBD
    static constexpr float ACC_RECAL = 200;
// Led
    Led m_led; // la led reflète l'état de l'asserv. Seule la machine a état en a besoin
};
#endif
