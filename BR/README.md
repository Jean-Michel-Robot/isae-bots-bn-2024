
# Trajectoire
Trajectoire désirée (θd (t), xd (t), yd (t))
θd_point = ωd , xd_point = vd cos(θd), yd_point = vdsin(θd)


Obtention de v_d et w_d à partir du point objectif :
vd = Rsb * pd_point
ωd = θd_point (0 en ligne droite)



Calcul de e = (ex, ey, eθ) avec :
Rsb * (pd - p) pour ex et ey
θ - θd pour eθ


Réglage des gains k1, k2, k3 avec la linéarisation, cad avec ωd et vd constants
Alors on peut stabiliser avec :

u 1 = −k1 |vd| ex
u 2 = −k2 vd ey − k3 |vd| eθ

Avec u1 = v coseθ − vd , u2 = ω − ωd

Puis utilisation de la formule de calcul de v et ω en non linéaire (vd variable)

On a alors v et ω, on peut les transformer en vL et vR



# OU avec le PID

v = vd + Kp*e + Ki*integr(e(tau)dtau)
θ = θd + Kp*eθ




# Point trajectoire


V* variable par échelons


Retourner à un temps t donné :
- x(t), y(t), theta(t) (/!\ t est le temps et pas le paramètre s de la courbe)
- v(t) et w(t) (dans le repère du robot)
	-> pour une ligne droite v vaut V* sauf pendant les rampes, et w est nul (theta constant)
		(dans le cas général v = sqrt(x.^2 + y.^2), avec le bon signe (v<0 pour la marche arrière)
		
La rampe doit d'effecter sur le paramètre s directement, puis on dérive toutes les vars à retourner à partir de s

Sans les rampes et en ligne droite :
Dtotale = ...
d_current = 
s(t) = d_current / Dtotale




# Com asserv - sm

La SM est notifiée d'un ordre par ROS et l'envoi d'un event
La SM choisit la trajectoire adaptée
La SM lance la fonction de suivi de l'asserv
La SM lance la rampe consigne
La SM update la rampe puis l'asserv régulièrement
La trajectoire envoie l'event de fin de rampe quand la distance restante est suffisamment faible
La trajectoire se marque comme terminée lorsque s est à 1
La SM peut alors transitionner


**L'asserv ne s'occupe pas de la trajectoire, que du suivi du point**
