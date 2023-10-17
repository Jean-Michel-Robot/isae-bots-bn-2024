# DISPLACEMENT NODE

Package du Displacement Node.

## CONTENT

* displacement_node.py
* dsp_comm.py
* dsp_util.py
* dsp_gain.py
* pathfinder/
* pathfinder_data/

## FUNCTIONING OF DispN

# PATHFINDER

* Travail sur l'A* remis à plus tard
* Node --> Structure de ce qu'est un noeud pour la grille
* Maps --> Terrain de jeu avec nodes de passage, nodes d'évitement et obstacles (c'est la map koi)
* Nodes Creators --> A partir de la grille dans pathfinder_data, génère les nodes de parcours
* Les obstacles --> Cercle, Triangle, Rect 
* Obs creator --> Obs connus rentrés à la main
* Grid --> Génération de grilles format xml
* Grid Creator --> Dans pathfinder_data : grâce aux données génère la grille

# Disp Utils

* patchFrameBR : pas besoin si BR ok
* toRobotCoord : passage dans repère loc du robot
* Vaguement des fonctions de mise en forme de LOGS

# Disp Gains

* WTF + coder avec le cul + Aled + Osef

# Disp Comm

* Déf de cstes + dico pour la comm avec Strat et Teensy 
* callback_teensy pour savoir comment ça se passe
* callback_strat : pk publier le current path || Pour recalage on devrait pas gérer les gains mais juste pub un truc pour dire au bn de se mettre dans ce mode || Faut tout refaire mdrr
* Manière de faire pas claire et à revoir

# Disp Node

*
