# Structure Bas Niveau pour la pami 


[![forthebadge](http://forthebadge.com/images/badges/built-with-love.svg)](http://forthebadge.com)  [![forthebadge](https://forthebadge.com/images/badges/made-with-c-plus-plus.svg)](https://forthebadge.com)

Code intégrale des pamis pour l'années 2024-2025

## Pour commencer

Readme afin d'aider à debugger les erreurs de compilation et de liens liées à la pami

### Pré-requis

* ESP 32 
* Capteur VL53L5X

### Stratégie 

La pami prend en entrée une position de départ (posx_init,ypos_y_init) et une position d'arrivée 
(pos_x_fin,pos_y_fin)et se déplace de manière autonome en ligne droite jusqu'à la position d'arrivée.
Si un obstacle est détecté, la pami s'arrête Voir ['machine_etats.cpp'](src/Machine_etat.cpp) pour plus de détails

Stratégie d'évitement très simple  a modifier selon besoin dans ['machine_etats.h'](src/Machine_etat.cpp)

## BUG et robustesse 
> La pami est robuste et peut être utilisée en compétition. Cependant, il est possible que des erreurs surviennent lors de l'utilisation de la pami. Voici quelques erreurs possibles et comment les résoudre :
Si la pami ne demarre pas au lancement de la tirette, erreurs possibles :
### Bugs très fréquents
- La tirette est mal branchée/branchée à l'envers : tester l'éat de la tirette en affichant son état de la sur le moniteur série
> Le port I2C est fragile, il est possible que le capteur de distance ne soit pas reconnu par l'ESP32 ou que le capteur de distance ne fonctionne pas correctement (ie : renvoie des valeurs aberrantes ou nulles)
- Le capteur de distance est mal branché à l'I2C  : tester le capteur de distance en affichant la distance mesurée sur le moniteur série 
Testez le capteur de distance en utilisant 'irsensor.setup' uniquement avec irsensor.loop() pour voir si le capteur de distance fonctionne correctement. Si erreur I2C WRITE/READ alors l'I2C est mal branché" ou il y a un faux contact
### Bugs un peu plus rares mais posible
- les odomètres sont pt , lancer uniquement le mesure_pos.loop() pour voir si les odomètres fonctionnent correctement

# Explication fonctionnement de la pami
## But 
La pami part d'un point A , s'oriente dans la direction du point B et se déplace en ligne droite jusqu'à atteindre le point B (la trajectoire du roobot se met à jour pour gardder le point B en arrivée). Si un obstacle est détecté, la pami s'arrête.
Un méthode d'évitement d'obstacle est en cours d'implémentation dans la pami ( voir etat AVOID ) mais cele-ci n'est pas encore fonctionelle
## Machine d'état
La machine à état est le coeur de la pami, elle permet de gérer les différents états de la pami. C'est cette fonction qui est appelée dans le main pour faire fonctionner la pami.
La machine d'état est composée de 4 états principaux :
- INIT
- MVT
- STOP 
- END 
- *AVOID (en cours d'implémentation)*

### Explication de la mesure de la position 
La mesure de position de la pami se fait à l'aide d'odomètres. La classe encodeur [encodeur.h](src/encodeur.h) mesure le nombre de tours effectués par la roue du capteur ( PAS celle du robots). 
Puis, dans la classe mesure_pos [mesure_pos.h](src/mesure_pos.h), on calcule la position de la pami en fonction du nombre de tours effectués par les roues.

A partir du nombre de tour de la roue droite entre deux instant , on connait alors la distance parcourue par la roue droite *position_r* . On fait de même pour la roue gauche *Position_l*. Puis on mesure l'angle de rotation du robot en fonction de la différence de distance parcourue par les deux roues.

A partir de l'angle de rotation et de la distance parcourue, on peut alors calculer la position x et y de la pami (ainsi que sa vitesse)


>Encodeur->position_r,position_l->angle_theta->position_x,position_y,vitesse

### Une explication de l'ASSERV
L'asserv est une fonction qui permet de contrôler la vitesse et la direction de la pami. Elle est appelée dans l'état MVT et permet de faire avancer la pami en ligne droite. Nous avons décider de mélanger l'asservissement en angle et en position pour plus de précision. Ainsi , le resultat de la fonction asserv_global est une somme d'un asservissement en angle ( garder la direction voule) et en position x et y 

## Auteurs
* **Baptiste Merlau** _alias_[@Bmrl7](https://github.com/Bmrl7)
* **Adam Benabou** _alias_ [@aeomath](https://github.com/aeomath)

Lisez la liste des [contributeurs](https://github.com/isae-bots-bn-2024/contributors) pour voir qui à aidé au projet !


