#ifndef _SM_CALLBACK_H
#define _SM_CALLBACK_H

enum AsservCallback // TODO retour renvoyé vers le haut niveau
{
    OK_POS = 1,
    OK_TURN = 2,
    OK_REVERSE = 3,
    OK_RECAL = 4,

    OK_READY = 5,
    OK_IDLE = 6,

    ERROR_ASSERV = 0,
};

enum GoalType // type d'objectif recu par le haut niveau
{ 
	UNVALID_GOALTYPE = -1, // sert a rejeter les valeurs non conformes

	FINAL = 0,             // point final, avec orientation
	TRANS = 1,             // point transitoire, sans orientation finale
	ORIENT = 9,            // orientation seule sur place
    REVERSE = 8,           // marche arrière

    RECAL_FRONT = -2,      // recalage avant //TODO pas def pour cette année
    RECAL_BACK = 6,        // recalage arrière

	STOP  = 2,             // freinage d'urgence
	RESET = 3,             // reset de la position odometrique
	CONTROL = 4,           // controle en commande directe
};


// The state machine does not have to know how the callback is actually processed.
// This will (but does not have to) delegate to ROS2.
void sendCallback(AsservCallback callback);

#endif