# PMI ACTION

## Les Topics ROS

### Resistances

```/res_request``` permet de démarrer la mesure de résistance (valeur <=> couleur du robot)

```/res_feedback``` reçoit la valeur de résistance où :
- 0 : jaune
- 1 : violet
- 2 : croix
- 3 : échec

```/resistance_update``` pour mettre à jour les angles des bras des résistances tel que :
```
rostopic pub /arm_update std_msgs/Int16MultiArray "layout:
  dim:
  - label: ''
    size: 0
    stride: 0
  data_offset: 0
data:
 [0,90,0,90]" -1 
 ```
 
 avec les angles dans cet ordre :
 1. gauche, rentré
 1. gauche, déployé
 1. droite, rentré
 1. droite, déployé

Pour les bras chargés de faire basculer les carrés, les topics ```arm_request```, ```arm_feedback``` et ```arm_update``` s'utilisent de la mème manière.

### Statuette & Replique

```drop_stat_request``` ouvre la pince (feedback ```drop_stat_feedback```)

```grab_stat_request``` ferme la pince (feedback ```grab_stat_feedback```)

```drop_replic_request``` permet de livrer la replique (feedback : ```drop_replic_feedback```)






 


 