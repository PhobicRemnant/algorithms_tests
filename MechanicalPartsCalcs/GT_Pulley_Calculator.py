from math import pi
"""
This program calculates the diameter and desired angles for a GT timing pulley.
To ease the design process in CAD software.
"""

# Number of teeth desired
nTeeth = int(input("How many teeth does your pulley needs?"))
# Belt teeth pitch
pitch = float(input("What is the teeth pitch on the belt?"))
# Diameter of desired pulley
pulleyDia = round( (nTeeth*pitch)/pi, 3)
# Degrees per teeth
deg4teeth = (360.0)/nTeeth
# Belt teeth size
bTeethSize = 1.38
# Pulley teeth size
pTeethSize = 0.75
# Belt teeth circle radious
bTeethR = (pulleyDia/2) - bTeethSize
# Teeth circle radious
pTeethR = (pulleyDia/2) - bTeethSize + pTeethSize
# 2nd radious
r2 = 0.4
# 2nd curve Radious angle
ang2ndteeth = ((deg4teeth)*r2)/pitch

# Show inputs and results
print(nTeeth)
print(pitch)
print(pulleyDia)
print(deg4teeth)
print(bTeethR)
print(pTeethR)
print(ang2ndteeth)