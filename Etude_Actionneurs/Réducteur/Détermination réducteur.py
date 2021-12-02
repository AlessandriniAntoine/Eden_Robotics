from math import *
  
def round_up(n, decimals = 0):  
    multiplier = 10 ** decimals  
    return ceil(n * multiplier) / multiplier 
  
n=0.98

"""moteur"""
rpm_moteur =  52.8 # rpm
Vitesse_moteur = rpm_moteur*2*pi/60 # rad.s^-1
Couple_moteur = 0.5 # N.m
Puissance_moteur = Couple_moteur*Vitesse_moteur # W
print("\nCouple moteur : ",Couple_moteur," N.m" )
print("Vitesse moteur : ",rpm_moteur," rpm")
print("Puissance moteur : ",round_up(Puissance_moteur,2)," W")

"""sortie réducteur"""
Couple_sortie = 2.4 # N.m
reducteur = n*Couple_moteur/Couple_sortie
rpm_sortie = reducteur*rpm_moteur # rpm
vitesse_sortie = Vitesse_moteur*reducteur # rad^s-1
Puissance_sortie =  vitesse_sortie*Couple_sortie # X
print("\nCouple sortie : ",Couple_sortie," N.m" )
print("Vitesse sortie : ",rpm_sortie," rpm")
print("Puissance sortie : ",round_up(Puissance_sortie,2)," W\n")

print("\nRapport de reduction",round_up(1/reducteur,2),"\n")