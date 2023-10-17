"""Package contenant l'implementation du Pathfinder.

Le module principal pathfinder.py utilise l'algorithm A* pour la recherche de 
chemin dans une grille de noeuds.

Les composants de la grille sont definis par les modules maps.py et node.py
pour la grille et les noeuds, ainsi que par les modules d'obstacles pouvant 
etre des rectangles ou des cercles.

La gestion d'exception se fait avec le module excetions.py definissant deux 
exceptions: PathNotFoundError et TimeOutError.

Author: mathieu  - 2018
Update: mdario   - 2022"""