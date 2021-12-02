###Dossier Matlab
Contient tous les éléments nécessaires à la simulation du robot sous simscape multibody.

Contient les fichiers matlab suivants : 
1. Bras_DataFile_PLA.m : correspond à tous les paramètres du bras (rotations, translations entre les parties 
2. Etude_PLA.m : Permet de simuler le bras et de récuper les couples, puissance et vitesse de rotation ainsi que de créer les graphiques correspondants 
3. 3. Initilisation_PLA.m : A lancer en premier, permet de créer le robot nécessaire aux simulations 
4. 4. Simulation_PLA.m : Contient les points d'arrivée et de départ et le temps de chaque simulation.

Contient les fichiers simulinks suivants : 1. Bras_PLA.slx : modèle du bras avec des angles donnés 2. Moteurs_PLA.slx : modèle de la simulation 


###Dossier Réducteur
1. Reducteur_PLA.m : Calcul le réducteur nécéssaire connaissant le couple nécéssaire et le couple moteur
2. Détermination réducteur.py : pareil
3. Vérification réducteur.py : renvoie les caractéristique nécéssaire du moteur connaissant celle nécéssaire et le réducteur


###Fichier PDF
Synthése pour chaque moteur des simulations puis du moteur choisis
