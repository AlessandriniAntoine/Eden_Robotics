import numpy as np
from robotGeneration import *

plate = Plate(205,50,6)

    # get cavity files
gmsh.initialize()
gmsh.option.setNumber("Mesh.MeshSizeFactor",20)    # get cavity files

plate.createPoint()

gmsh.model.mesh.generate(3)
gmsh.fltk.run()
gmsh.finalize()