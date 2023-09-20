

# Ordres du HN sur les actionneurs

- Servo des cerises : Int16 sur /strat/cherries (0 : servo position haute | 1 : servo position basse)

- Servos des volets : Int16 sur /strat/doors (0 : les deux servos position fermée | 1 : les deux servos position ouverte)

- Servo de la pince des palets : Int16 sur /strat/clamp (0 : servo position fermée | 1 : servo position ouverte)

- Stepper de l'ascenseur : Int16 sur /strat/elevator (entier de 0 à 8 codant la hauteur de la pince en numéro de palet)
  Ex : 0 est la hauteur de la pince pour prendre le palet posé sur le sol, 1 la hauteur pour prendre le palet posé par-dessus, etc jusqu'à 8 (pile de 9 palets max)



# Callbacks du BN vers le HN

TODO