import numpy as np
from dataclasses import dataclass, field
import gmsh

@dataclass
class Robot:
    pass

########################
# body member
########################

@dataclass
class Base:

    def createPoints(self):
        pass

@dataclass
class Scapula: # omoplate
    pass

@dataclass
class Arm:
    pass

@dataclass
class Forearm:
    pass

@dataclass
class Hand:
    pass

########################
# Motors
########################

@dataclass
class XC430:
    radius : float = 2.5
    height : float = 40
    width : float = 22

@dataclass
class XM430:
    pass

@dataclass
class XL430:
    pass

########################
# articulation
########################

@dataclass
class Pelvis: # bassin
    pass

@dataclass 
class Shoulder: 
    pass

@dataclass
class Elbow:
    pass

@dataclass
class Wrist: # Poignet
    pass

########################
# Part
########################

@dataclass
class Plate:
    length : float 
    width : float
    thickness : float
    
    lc : float = 0.5

    def __post_init__(self,motor):
        self.motor = motor

    def createPlate(self):
        self.points = []
        self.points.append(gmsh.model.occ.addPoint(0, 0, 0, self.lc))
        self.points.append(gmsh.model.occ.addPoint(0, self.length, 0, self.lc))
        self.points.append(gmsh.model.occ.addPoint(self.width, self.length, 0, self.lc))
        self.points.append(gmsh.model.occ.addPoint(self.width, 0, 0, self.lc))

        self.lines = []
        for i in range(1,len(self.points)):
            self.lines.append(gmsh.model.occ.addLine(self.points[i-1],self.points[i]))
        self.lines.append(gmsh.model.occ.addLine(self.points[-1],self.points[0]))
        
        self.curve = gmsh.model.occ.addCurveLoop(self.lines)
        self.surface = gmsh.model.occ.addPlaneSurface([self.curve])
        self.volume = gmsh.model.occ.extrude([(2,self.surface)], 0, 0,self.thickness)
        gmsh.model.occ.synchronize()


@dataclass
class EngineShaft:

    length : float
