#Code asserv Base Roulante
Code de l'asservissement en position du gros robot, code utilisé pendant la coupe 2019

##Compilation 
Avec l'IDE arduino, pour une Teensy 3.2 avec les libs rosserial installées pour la version de ROS correspondant au robot.
Attention la convention de code est en c++11 et ne compilera pas en Arduino pur, qui est en c++9x de base (en plus des Décodeurs HW)

## Utilisation 

Tout d'abord lancer 
```
roscore
rosrun rosserial_arduino serial_node.py /dev/ttyBR _baud:=115200
```
Les gains et rampes sont set par défaut dans le code, on peut les modifier avec les topics appropriés

## Les entrees sur l'asserv

### Le topic nextPositionTeensy

Ce topic centralise toutes les consignes de position envoyées. Il s'agit d'un quaternion dont :

```
x,y,z : coordonnées x,y,theta du point
w : entier correspondant au type de consigne
```

Les consignes sont définies telles quelles :

```
0 : point final (marche avant)
1 : point transitoire (pas d'orientation finale)
2 : stop (progressif si le robot est en train de se déplacer, cf doc)
3 : reset de la position, bascule le robot en stop
4 : control, permet de controler directement les moteurs
5 : recalage avant : le robot s'oriente selon theta = w, puis avance en marche avant jusqu'au contact d'un mur et reset sa position (x ou y)
6 : recalage arriere : le robot s'oriente selon theta = w, puis recule en marche arriere jusqu'au contact
7 : light_final_avant : point final marche avant, mais avec une dynamique assouplie
8 : light_final_arriere : idem mais en marche arriere
9 : orientation pure vers l'angle theta = w
10 : fake_recal_avant : le robot se plaque au mur mais sans reset sa position
11 : fake_recal_arriere : idem
```
Pour contourner les bugs de terminaux, je conseille d'utiliser le script InputTeensyAsserv qui les évite (cf la partie scripts)

### Le topic speedTeensyObjective

Ce topic renseigne les vitesses max que le robot peut atteindre. Pour l'editer il faut la syntaxe suivante : 
```
rostopic pub  /speedTeensyObjective std_msgs/Float32MultiArray "{layout: {dim : [size: 2]} ,data: [700.0,100.0]}" 
```
où 700 est la vitesse lineaire en mm.s-1 et 100 la vitesse angulaire en rad.s-1

Ce topic est concu pour supporter une modification à tout moment à la volée, la dynamique du robot s'adapte (dans la limite du raisonnable physiquement parlant)

### Le topic dynamicParameters

Ce topic set les dynamiques demandées aux rampes. 
```
rostopic pub  /dynamicParameters2 std_msgs/Float32MultiArray "{layout: {dim : [size: 8]} ,data: [0,1,2,3,4,5,6,7,8]}" -1

Avec dans l'ordre :
- acceleration, deceleration brutale, deceleration nominale des rampes lineaires rapides
- acceleration, deceleration brutale, deceleration nominale des rampes lineaires smooth
- acceleration,deceleation des rampes en rotation rapides
```

L'ancien topic est gardé par compatibilité :

```
rostopic pub  /dynamicParameters std_msgs/Float32MultiArray "{layout: {dim : [size: 5]} ,data: [0,1,2,3,4,5]}" -1

Avec dans l'ordre :
- acceleration, deceleration brutale, deceleration nominale des rampes lineaires rapides
- acceleration,deceleation des rampes en rotation rapides
```
### Le topic gainsMotor 

Ce topic regle les gains de l'asserv Moteur
```
rostopic pub  /gainsMotor std_msgs/Float32MultiArray "{layout: {dim : [size: 4]} ,data: [Kf,Kp,Ti,Td]}" -1

```

### Le topic gains
Ce topic regle les gains des asserv avance et rotation :
```
rostopic pub  /gains std_msgs/Float32MultiArray "{layout: {dim : [size: 6]} ,data: [Kp_pos,Ti_pos,Td_pos,Kp_angle,Ti_angle,Td_angle]}" -1

```


## Les feedbacks 

### Le topic current_position

Ce topic retourne la position actuelle du robot toutes les 50ms 

### Le topic okPosition

Ce topic effectue le feedback des étapes de l'asservissement. Le message est un entier parmi :
```
1 : okPos, le robot a fini son avance lineaire
2 : okTurn, le robot a fini de tourner 
3 : le robot va en marche arriere (a supprimer)
4 : le robot va en marche avant (a supprimer)
0 : erreur dans l'asserv (timeout, derapage, divergence etc...) donc erreur qui peut reussir apres un deuxieme essai
-1 : erreur bloquante (gains non sets ou non valides, erreurs internes etc...)
```
Dans le cas des erreurs, le topic /logDebug donne plus de details

### Le topic logTotaleArray 

Ce topic effectue un log global des variables internes de l'asserv. Pour l'exploiter :
```
rostopic echo -p /logTotaleArray > fichierdelog.log
python GraphPos.py fichierdelog.log

Si la teensy est connectée a la pi, un scp pi@ip:fichierdelog.log . permet de le recuperer
@Yoann a concu un affichage a la volée de ces log
```


## Les scripts 

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
export ROS_IP=192.168.43.12
export ROS_MASTER_URI=http://192.168.43.12:11311
```
Ou alors utiliser les scripts export_roscore.sh et export_rosrun.sh sur la pi

Sur le pc fixe 
```
export ROS_IP=192.168.43.139 #IP perso
export ROS_MASTER_URI=http://192.168.43.12:11311
cd Robotik/Robotik_2019/Sources/Robot/Haut_niveau/Dev/
source devel/setup.bash
rosrun isae_robotics_graph GraphNode.py 
rosrun isae_robotics_graph InterfaceNode.py 
```
Ou utiliser export_costa_launcher.sh ou export_titanic_launcher.sh dans le dossier Scripts
