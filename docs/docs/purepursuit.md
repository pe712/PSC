Fonctionnement théorique de l'algorithme de pure poursuite
*  Déterminer la position actuelle du véhicule sur le circuit
*  Trouver le point de l’itinéraire le plus proche du véhicule
*  Trouver le point d’objectif
*  Faire la transformation pour le déterminer dans le référentiel du véhicule
*  Trouver la courbure nécessaire pour que le véhicule atteigne ce point
*  Envoyer la requête au véhicule pour qu’il modifie sa trajectoire
*  Mettre à jour les coordonnées du véhicule

Formule pour la pure poursuite :
Gamma = 2x/l²

Vous pouvez voir l'implémentation [ici](https://github.com/pe712/PSC/blob/main/node/pure_pursuit.py)
