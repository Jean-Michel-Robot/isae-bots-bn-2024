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




## Auteurs
* **Baptiste Merlau** _alias_[@Bmrl7](https://github.com/Bmrl7)
* **Adam Benabou** _alias_ [@aeomath](https://github.com/aeomath)

Lisez la liste des [contributeurs](https://github.com/isae-bots-bn-2024/contributors) pour voir qui à aidé au projet !


