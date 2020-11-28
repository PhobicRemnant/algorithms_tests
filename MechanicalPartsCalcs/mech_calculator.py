from math import pi

def GT2Pulley():
    
    """
    This GT2 Pulley is based on a schematic profile found in: http://www.handsontec.com/dataspecs/gt2-belt-B.pdf
    Common appliances are small CNC machines like 3D printers.
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

    # Return needed values as a tuple to add to picture
    return (pulleyDia, deg4teeth, bTeethR, pTeethR, ang2ndteeth)