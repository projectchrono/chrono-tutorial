# =============================================================================
# PROJECT CHRONO - http:#projectchrono.org
#
# Copyright (c) 2014 projectchrono.org
# All right reserved.
#
# Use of this source code is governed by a BSD-style license that can be found
# in the LICENSE file at the top level of the distribution and at
# http:#projectchrono.org/license-chrono.txt.
#
# =============================================================================
# Author: Simone Benatti
# =============================================================================
#
# Create a falling cable using FEA module (FEA tutorial n.1)
#
# This cable is made with N beam elements of ChElementANCFcable type. They are
# added to a ChMesh and then the first node is connected to the absolute
# reference using a constraint.
#
# The cable falls under the action of gravity alone, acting in the negative
# Y (up) direction.
#
# The simulation is animated with Irrlicht.
#
# =============================================================================

import pychrono as chrono
import pychrono.fea as fea
import pychrono.irrlicht as chronoirr

# 0. Set the path to the Chrono data folder
CHRONO_DATA_DIR = "E:/Repositories/chrono/data/"
chrono.SetChronoDataPath(CHRONO_DATA_DIR)

# 1. Create the physical system that will handle all finite elements and constraints.

#    Specify the gravitational acceleration vector, consistent with the
#    global reference frame having Y up (ISO system).
system = chrono.ChSystemNSC()
system.SetGravitationalAcceleration(chrono.ChVector3d(0, -9.81, 0))


# 2. Create the mesh that will contain the finite elements, and add it to the system

mesh = fea.ChMesh()

system.Add(mesh)


# 3. Create a material for the beam finite elements.

#    Note that each FEA element type requires some corresponding
#    type of material. Here we will use ChElementCableANCF elements:
#    they use a material of type ChBeamSectionCable, so let's do

beam_material = fea.ChBeamSectionCable()
beam_material.SetDiameter(0.01)
beam_material.SetYoungModulus(0.01e9)
beam_material.SetBeamRaleyghDamping(0.01)


# 4. Create the nodes

#    - We use a simple for() loop to create nodes along the cable.
#    - Nodes for ChElementCableANCF must be of ChNodeFEAxyzD class
#      i.e. each node has 6 coordinates: position, direction, where
#      direction is the tangent to the cable.
#    - Each node must be added to the mesh, ex.  mesh.Add(my_node)
#    - To make things easier in the following, we store node pointers
#      into an optional 'beam_nodes' array, i.e. a std::vector<>, later we
#      can use such array for easy creation of elements between the nodes.

beam_nodes = []

length = 1.2  # beam length, in meters
N_nodes = 16
for i_ni in range(N_nodes)  :
    # i-th node position
    position = chrono.ChVector3d(length * (i_ni / (N_nodes - 1)),  # node position, x
                                0.5,                                  # node position, y
                                0)                                   # node position, z

    # i-th node direction
    direction = chrono.ChVector3d(1.0, 0, 0)

    # create the node
    node = fea.ChNodeFEAxyzD(position, direction)

    # add it to mesh
    mesh.AddNode(node)

    # add it to the auxiliary beam_nodes
    beam_nodes.append(node)



# 5. Create the elements

#    - We use a simple for() loop to create elements between the
#      nodes that we already created.
#    - Each element must be set with the ChBeamSectionCable material
#      that we already created
#    - Each element must be added to the mesh, ex.  mesh.Add(my_element)

for ie in range(N_nodes - 1) :
    # create the element
    element = fea.ChElementCableANCF()

    # set the connected nodes (pick two consecutive nodes in our beam_nodes container)
    element.SetNodes(beam_nodes[ie], beam_nodes[ie + 1])

    # set the material
    element.SetSection(beam_material)

    # add it to mesh
    mesh.AddElement(element)



# 6. Add constraints

#    - Constraints can be applied to FEA nodes
#    - For the ChNodeFEAxyzD there are specific constraints that
#      can be used to connect them to a ChBody, namely
#      ChLinkNodeFrame and ChLinkNodeSlopeFrame
#    - To attach one end of the beam to the ground, we need a
#      'truss' ChBody that is fixed.
#    - Note. An alternative, only when the node must be fixed 
#      to absolute reference, is not using constraints, and just
#      use: beam_nodes[0].SetFixed(True)  (but would fix also dir)

truss = chrono.ChBody()
truss.SetFixed(True)
system.Add(truss)

# lock an end of the wire to the truss
constraint_pos = fea.ChLinkNodeFrame()
constraint_pos.Initialize(beam_nodes[0], truss)
system.Add(constraint_pos)


## -------------------------------------------------------------------------
## EXERCISE 1a
##
## Add a cylinder.
## Suggested size: 0.02 radius, 0.1 height, density:1000.
## Hint: use the ChBodyEasyCylinder to make the cylinder, pass size as 
## parameters in construction.
## 
## -------------------------------------------------------------------------

# create the cylinder
cylinder = chrono.ChBodyEasyCylinder(
            chrono.ChAxis_Y, # cylinder alignment
			0.02,            # radius
			0.1,             # height
			1000,            # density (used to auto-set inertia, mass)
			True,            # do collide 
			True)            # do visualize

# move cylinder to end of beam
cylinder.SetPos( beam_nodes[-1].GetPos() + chrono.ChVector3d(0, -0.05, 0) )

# add it to the system
system.Add(cylinder)

## -------------------------------------------------------------------------
## EXERCISE 1b
##
## Attach the cylinder to the free end of the cable.
## Hint: use the ChLinkNodeFrame to connect the cylinder and the end node.
## 
## -------------------------------------------------------------------------

# lock an end of the wire to the cylinder
constraint_cyl = fea.ChLinkNodeFrame()
constraint_cyl.Initialize(beam_nodes[-1], cylinder)
system.Add(constraint_cyl)


# 7. Make the finite elements visible in the 3D view

#   - FEA fisualization can be managed via an easy
#     ChVisualizationFEAmesh helper class.
#     (Alternatively you could bypass this and output .dat
#     files at each step, ex. for VTK or Matalb postprocessing)
#   - This will automatically update a triangle mesh (a ChVisualShapeTriangleMesh
#     asset that is internally managed) by setting proper
#     coordinates and vertex colours as in the FEA elements.
#   - Such triangle mesh can be rendered by Irrlicht or POVray or whatever
#     postprocessor that can handle a coloured ChVisualShapeTriangleMesh).
#   - Do not forget AddAsset() at the end!

mvisualizebeamA = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamA.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_ANCF_BEAM_AX)
mvisualizebeamA.SetColorscaleMinMax(-0.005, 0.005)
mvisualizebeamA.SetSmoothFaces(True)
mvisualizebeamA.SetWireframe(False)
mesh.AddVisualShapeFEA(mvisualizebeamA)

mvisualizebeamC = chrono.ChVisualShapeFEA(mesh)
mvisualizebeamC.SetFEMglyphType(chrono.ChVisualShapeFEA.GlyphType_NODE_DOT_POS)
mvisualizebeamC.SetFEMdataType(chrono.ChVisualShapeFEA.DataType_NONE)
mvisualizebeamC.SetSymbolsThickness(0.006)
mvisualizebeamC.SetSymbolsScale(0.005)
mvisualizebeamC.SetZbufferHide(False)
mesh.AddVisualShapeFEA(mvisualizebeamC)


# 8. Configure the solver and timestepper

#    - the default SOLVER_SOR of Chrono is not able to manage stiffness matrices
#      as required by FEA! we must switch to a different solver.
#    - We pick the SOLVER_MINRES solver and we configure it.
#    - Note that if you build the MKL module, you could use the more precise MKL solver.

# Change solver
solver = chrono.ChSolverMINRES()
solver.SetMaxIterations(200)
solver.SetTolerance(1e-10)
solver.EnableWarmStart(True)
system.SetSolver(solver)

# Change integrator:
system.SetTimestepperType(chrono.ChTimestepper.Type_EULER_IMPLICIT_LINEARIZED)  # default: fast, 1st order
# system.SetTimestepperType(ChTimestepper::Type::HHT)  # precise, slower, might iterate each step


# 9. Prepare visualization with Irrlicht
#    Note that Irrlicht uses left-handed frames with Y up.

# Create the Irrlicht application and set-up the camera.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.SetWindowSize(1024, 768)
vis.SetWindowTitle("FEA cable collide demo")
vis.Initialize()
vis.AddLogo()
vis.AddSkyBox()
vis.AddTypicalLights()
vis.AddCamera(chrono.ChVector3d(0.1, 0.2, -2.0))
vis.SetSymbolScale(0.1)
vis.AttachSystem(system)


# 10. Perform the simulation.

# Specify the step-size.
step_size = 0.001
realtime_timer = chrono.ChRealtimeStepTimer()

while vis.Run():
    vis.BeginScene() 

    # Render Chrono item assets
    vis.Render()

    # Draw an XZ grid at the global origin to add in visualization.
    chronoirr.drawGrid(
        vis, 0.1, 0.1, 20, 20,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_PI_2)),
        chrono.ChColor(0.4, 0.7, 0.4), True)

    vis.EndScene()

    ## Advance simulation by one step
    system.DoStepDynamics(step_size)

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)

