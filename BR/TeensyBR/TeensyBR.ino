/*
     ____                                                  
    / ___| _   _ _ __   __ _  ___ _ __ ___                 
    \___ \| | | | '_ \ / _` |/ _ \ '__/ _ \                
     ___) | |_| | |_) | (_| |  __/ | | (_) |               
    |____/ \__,_| .__/ \__,_|\___|_|  \___/                
   ____       _ |_|       _   _ _       ____ _       _     
  |  _ \ ___ | |__   ___ | |_(_) | __  / ___| |_   _| |__  
  | |_) / _ \| '_ \ / _ \| __| | |/ / | |   | | | | | '_ \ 
  |  _ < (_) | |_) | (_) | |_| |   <  | |___| | |_| | |_) |
  |_| \_\___/|_.__/ \___/ \__|_|_|\_\  \____|_|\__,_|_.__/ 


This code is to be uploaded on the TeensyBR
It is to be full C, with a cyclic call to functions at a given period


**** DOC ****

Penser une structure en blocs de calcul qui se suivent (comme Matlab) -> peuvent prendre en entrée la sorte du bloc suivant
- Struct contexte (vars intermédiaires et params)
- Struct entrée
- Struct sortie"

Les fonctions utilisent les strucuture de cette manière :
fct(entree, contexte, *sortie)

il faut faire les opérations sur les blocs dans le bon ordre mais y a pas de concurrence de tâches
→ revient à faire un schéma simulink

Follow the carrot pour l’asserv pos
Un point de contrôle pas au centre du robot qui cherche à aller sur un point visé (qui se déplace à la vitesse de consigne)
Pour l’instant le point visé fait des trajectoires simples (rot lin rot)
→ mail mathieu qui a de la doc

*************

*/
