from math import *
  
def round_up(n, decimals = 0):  
    multiplier = 10 ** decimals  
    return ceil(n * multiplier) / multiplier 

n=0.98
reducteur = 5 ;

"""sortie réducteur"""
Couple_sortie = 2.4 # N.m
rpm_sortie = 9 # rpm
Vitesse_sortie = rpm_sortie*2*pi/60 # rad^s-1
Puissance_sortie =  Vitesse_sortie*Couple_sortie # X
print("\nCouple sortie : ",Couple_sortie," N.m" )
print("Vitesse sortie : ",rpm_sortie," rpm")
print("Puissance sortie : ",round(Puissance_sortie,2)," W\n")

"""moteur"""
rpm_moteur = rpm_sortie*reducteur  # rpm
Vitesse_moteur = rpm_moteur*2*pi/60 # rad.s^-1
Couple_moteur = Couple_sortie/(n*reducteur) # N.m
Puissance_moteur = Couple_moteur*Vitesse_moteur # W
print("\nCouple moteur : ",Couple_moteur," N.m" )
print("Vitesse moteur : ",rpm_moteur," rpm")
print("Puissance moteur : ",round(Puissance_moteur,2)," W\n")

# vitesse = 2
# couple = 5.2
# # vitesse = 0.3*vitesse + vitesse 
# print("\nVitesse :",vitesse)
# couple = 0.3*couple + couple
# print("Couple : ",couple)
# puissance = vitesse*2*pi/60*couple
# print("Puissance:",round_up(puissance,2),"\n")