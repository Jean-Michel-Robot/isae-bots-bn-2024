# Code asserv Base Roulante
Code de l'asservissement en position du gros robot avec un noeud ROS2 Jazzy.
L'asservissement est un PID avec filtre passe-bas sur la dérivée. L'accélération maximale du robot est contrôlée par des rampes.
Les gains du PID sont définies avec une valeur par défaut dans le code, on peut les modifier avec les topics appropriés (la modification n'est valable que jusqu'au redémarrage de la Teensy).

Le fichier br_controller/include/configuration.hpp contient la plupart des paramètres de l'asservissement, s'il y a besoin de les modifier de manière permanente.

## Compilation 

Attention : la racine du projet Platform.IO est dans le dossier br_controller, et il y a un lien symbolique vers br_messages dans br_controller/extra_packages.
Cela était nécessaire pour que le code soit compatible à la fois avec colcon et Platform.IO.

### Compiler la simulation
Se placer à la racine du projet, dans un environnement où RO2 Jazzy est installé (par exemple le Docker du HN), puis lancer:
```
colcon build
```
L'exécutable est à l'emplacement ./install/br_simu/lib/br_simu/simulation_br, mais il faut sourcer "install/setup.bash" ou "install/local_setup.bash" avant de lancer la simulation, ce qui implique
de distribuer tout le dossier install avec l'exécutable.

### Compiler pour la BR (Arduino Teensy)
Compiler avec Platform.IO.
Attention la convention de code est en c++20 et ne compilera pas en Arduino pur, qui est en c++9x de base (en plus des Décodeurs HW)

## Utilisation (sur le robot)
TODO
rosserial a été remplacé par micro_ros, mais on n'a pas encore eu l'occasion de tester sur le robot.

## Les entrees sur l'asserv

Tous les topics sont exactement les mêmes entre la simulation et le vrai robot, sauf pour le topic "odos_count".

### Le topic nextPositionTeensy

Ce topic centralise toutes les consignes de position envoyées. Il s'agit d'un quaternion dont :

```
x,y,z : coordonnées x,y,theta du point
w : entier correspondant au type de consigne
```

Les consignes sont définies telles quelles :

```
0 : point final (marche avant)
1 : point transitoire (pas d'orientation finale, theta est ignoré)
2 : stop (progressif si le robot est en train de se déplacer, cf doc)
3 : reset de la position, bascule le robot en stop et réinitalise le PID
4 : [SUPPRIME] permet de controler directement les moteurs
5 : [PAS ENCORE IMPLEMENTE] recalage avant : le robot s'oriente selon theta = z, puis avance en marche avant jusqu'au contact d'un mur et reset sa position (x ou y)
6 : [PAS ENCORE IMPLEMENTE] recalage arriere : le robot s'oriente selon theta = z, puis recule en marche arriere jusqu'au contact
7 : [SUPPRIME] point final marche avant, mais avec une dynamique assouplie
8 : [SUPPRIME] idem mais en marche arriere
9 : orientation pure vers l'angle theta = z (x et y sont ignorés)
10 : [SUPPRIME] fake_recal_avant : le robot se plaque au mur mais sans reset sa position
11 : [SUPPRIME] fake_recal_arriere : idem
```
Les consignes marquées [SUPPRIME] ne seront pas réimplémentées sauf si elles sont vraiment nécessaires.

### Le topic obstacle_seen
Ce topic réduit temporairement la vitesse du robot (par exemple à l'approche d'un obstacle).
La valeur à donner est un entier entre 1 et 100 qui correspond à la nouvelle vitesse limite du robot, exprimée en pourcentage de la vitesse maximale globale.
La limitation est appliquée uniquement au déplacement en cours. Si aucun déplacement n'est en cours, le topic n'a aucun effet.
```
ros2 topic pub /teensy/obstacle_seen std_messages/Int16 "{data: <percentage>}"
```

NOTE: Il faudrait changer le nom de ce topic, mais l'ancien nom a été conservé pour le moment, pour des raisons de compatibilité avec le HN.

### Le topic gains
Ce topic regle les gains du PID:
```
ros2 topic pub /gains br_messages/GainsPid "{kp: <value>
ti: <value>
td: <value>}"
```
Publier sur ce topic réinitialise la valeur de l'intégrale et de la dérivée de l'erreur dans le PID.

## Les feedbacks 

### Le topic current_position

Ce topic retourne la position actuelle du robot toutes les 100ms (ou toutes les 10ms pour la simulation)

### Le topic okPosition

Ce topic effectue le feedback des étapes de l'asservissement. Le message est un entier parmi :
```
1 : okPos, le robot a fini son avance lineaire
2 : okTurn, le robot a fini de tourner 
3 : le robot a fini une marche arrière (ce code est toujours immédiatement suivi de okPos)
0 : [jamais envoyé actuellement] erreur dans l'asserv (timeout, derapage, divergence etc...) donc erreur qui peut reussir apres un deuxieme essai
-1 : [jamais envoyé actuellement] erreur bloquante (gains non sets ou non valides, erreurs internes etc...)
```

### Le topic logTotaleArray 

NB: L'asserv publie sur ce topic uniquement si la constante `_DEBUG` est définie lors de la compilation (elle est définie par défaut, voir br_controller/CMakeLists.txt et br_controller/platformio.ini pour la désactiver).

Ce topic effectue un log global des variables internes de l'asserv. Pour l'exploiter :
```
ros2 topic echo -p /logTotaleArray > fichierdelog.log
python GraphPos.py fichierdelog.log

Si la teensy est connectée a la pi, un scp pi@ip:fichierdelog.log . permet de le recuperer
@Yoann a concu un affichage a la volée de ces log
```

## Possible erreurs à detecter **Avant** la coupe 

- Tester asserv en idle : --> Topic br/idle 
- Tester asserv en Position --> Topic nextPositionTeensy. Tester tous les angles et les directions puis adapter si besoin les paramètres dans le fichier ['configuration.hpp'](br_controller/include/configuration.hpp).

- Si le robot va trop vite ou a une trop forte accélération, il y a un risque de glissement et de dérive de la position estimée par les odomètres. Penser à vérifier que les vitesses et accélérations sont adaptées (cela inclut l'accélération du freinage d'urgence). Si la position estimée n'est pas suffisamment précise, la stratégie du haut niveau ne fonctionnera pas.

### IMPORTANT
**Refaire le test après avoir redemarré le robot pour vérifier que les variables se sont bien initialisées !**
*Il est arrivé dans les versions précédentes du code que après un redémarrage de la teensy , certaines variables de trajectoires ne soient pas bien initialisées et donc le robot ne bouge pas ou devient fou (calcul de trajectoire infini)*

L'interface graphique (dans le code du haut niveau) permet d'envoyer un ordre de déplacement de type "0 : point final (marche avant)" au topic nextPositionTeensy et de visualiser le retour de position sur le topic okPosition.
Il suffit de faire un clic droit à l'emplacement où on veut faire aller le robot, puis de déplacer la souris dans la direction que l'on veut pour l'orientation finale avant de relacher le clic.

Sur le Docker HN :
```
source /app/install/setup.bash
ros2 run uix interface_node
```
(Penser à faire `xhost +` si besoin avant d'aller dans le Docker)

## [TOUT CE QUI SUIT EST POTENTIELLEMENT OBSOLETE, TODO à vérifier]
## Les scripts 
[Où sont les scripts ?]

Pour se simplifier la tache, plusieurs scripts sont disponibles dans le dossier Script

### InputTeensyAsserv

Reformule simplement un envoi sur le topic nextPositionTeensy, on entre simplement x,y,theta et le type d'ordre

### rotation.py

Fais effectuer 10 tours sur lui-meme au robot, utile pour regler l'odometrie

### SetGainsTeensy

Permet de régler tous les gains de la teensy, utile pendant les reglages

### SpamSpeed 
spam les ordres de consigne pour tester la robustesse

### suite_point.py

Permet d'effectuer une suite de points, simulation d'une trajectoire avec un path-finder haut niveau

### GraphPos.py

Effectue le tracé des logs de l'asserv 

### Dossier plot

Contient tous les scripts pour effectuer des logs et les tracer.
Les noms sont explicites : 
- acquireLogs est à lancer SUR la pi
- les autres se lancent sur l'ordinateur où on effectue les plots
- getAndPlotLastLog va automatiquement chercher le dernier log sur la pi, le télécharge et le plot

NOTE : Les fichiers sont effacés au redémarrage de la pi/de l'ordinateur, peut-être à modifier :-D

## utiliser les graphes en Live

Pour afficher les graphes en live, qques commandes a lancer en plus : 

Sur la Pi (dans le meme terminal que roscore et rosrun): 
```
export ROS_IP=192.168.217.11
export ROS_MASTER_URI=http://192.168.217.11:11311
```
Ou alors utiliser les scripts export_roscore.sh et export_rosrun.sh sur la pi

Sur le pc fixe 
```
#IP perso ici
export ROS_IP=192.168.217.131
export ROS_MASTER_URI=http://192.168.217.11:11311
cd Robotik/Robotik_2019/Sources/Robot/Haut_niveau/Dev/
source devel/setup.bash
rosrun isae_robotics_graph GraphNode.py 
rosrun isae_robotics_graph InterfaceNode.py 
```
Ou utiliser export_costa_launcher.sh ou export_titanic_launcher.sh dans le dossier Scripts

