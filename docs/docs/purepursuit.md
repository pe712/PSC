Fonctionnement théorique de l'algorithme de pure poursuite
# Pure Pursuit
1. Déterminer la position actuelle du véhicule sur le circuit
2. Trouver le point de l’itinéraire le plus proche du véhicule
3. Trouver le point d’objectif
4. Faire la transformation pour le déterminer dans le référentiel du véhicule
5. Trouver la courbure nécessaire pour que le véhicule atteigne ce point
6. Envoyer la requête au véhicule pour qu’il modifie sa trajectoire
7. Mettre à jour les coordonnées du véhicule

Formule pour la pure poursuite :
Gamma = 2x/l²

Vous pouvez voir l'implémentation [ici](https://github.com/pe712/PSC/blob/main/node/pure_pursuit.py)
