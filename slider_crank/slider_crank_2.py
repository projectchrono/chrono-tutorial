## =============================================================================
## PROJECT CHRONO - http:##projectchrono.org
##
## Copyright (c) 2014 projectchrono.org
## All right reserved.
##
## Use of this source code is governed by a BSD-style license that can be found
## in the LICENSE file at the top level of the distribution and at
## http://projectchrono.org/license-chrono.txt.
##
## =============================================================================
## Author: Simone Benatti
## =============================================================================
##
## Slider-crank Chrono tutorial (model 2)
##
## This model is a 3-body slider-crank consisting of crank, slider and connecting
## rod bodies. The crank is connected to ground with a revolute joint and the
## slider is connected to ground through a prismatic joint.  The connecting rod
## connects to the crank through a spherical joint and to the slider through a
## universal joint.
##
## The crank body is driven at constant angular speed, under the action of gravity,
## acting in the negative Z direction.
##
## An additional spherical body, constrained to move along the global X axis
## through a prismatic joint and connected to ground with a translational spring
## damper, interacts through contact with the slider body.
##
## The simulation is animated with Irrlicht.
##
## =============================================================================

import pychrono as chrono
from pychrono import irrlicht as chronoirr

## 0. Set the path to the Chrono data folder
chrono.SetChronoDataPath('E:/Repositories/chrono/data/')

## 1. Create the physical system that will handle all bodies and constraints.

##    Specify the gravitational acceleration vector, consistent with the
##    global reference frame having Z up.
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVector3d(0, 0, -9.81))

## 2. Create the rigid bodies of the slider-crank mechanical system.
##    For each body, specify:
##    - a unique identifier
##    - mass and moments of inertia
##    - position and orientation of the (centroidal) body frame
##    - visualization assets (defined with respect to the body frame)

## Ground
ground = chrono.ChBody()
system.AddBody(ground)
ground.SetIdentifier(-1)
ground.SetName("ground")
ground.SetBodyFixed(True)

cyl_g = chrono.ChVisualShapeCylinder(0.03, 0.4)
cyl_g.SetColor(chrono.ChColor(0.6, 0.6, 0.2))
ground.AddVisualShape(cyl_g)

## Crank
crank = chrono.ChBody()
system.AddBody(crank)
crank.SetIdentifier(1)
crank.SetName("crank")
crank.SetMass(1.0)
crank.SetInertiaXX(chrono.ChVector3d(0.005, 0.1, 0.1))
crank.SetPos(chrono.ChVector3d(-1, 0, 0))
crank.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))

box_c = chrono.ChVisualShapeBox(1.9, 0.1, 0.1)
crank.AddVisualShape(box_c)

cyl_c = chrono.ChVisualShapeCylinder(0.05, 0.2)
crank.AddVisualShape(cyl_c, chrono.ChFramed(chrono.ChVector3d(1, 0, 0), chrono.QuatFromAngleX(chrono.CH_C_PI_2)))

sph_c = chrono.ChVisualShapeSphere(0.05)
sph_c.SetColor(chrono.ChColor(0.6, 0.2, 0.2))
crank.AddVisualShape(sph_c, chrono.ChFramed(chrono.ChVector3d(-1, 0, 0)))

## Slider
slider = chrono.ChBody()
system.AddBody(slider)
slider.SetIdentifier(2)
slider.SetName("slider")
slider.SetMass(1.0)
slider.SetInertiaXX(chrono.ChVector3d(0.05, 0.05, 0.05))
slider.SetPos(chrono.ChVector3d(2, 0, 0))
slider.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))

box_s = chrono.ChVisualShapeBox(0.4, 0.2, 0.2)
box_s.SetColor(chrono.ChColor(0.2, 0.2, 0.6))
slider.AddVisualShape(box_s)

cyl_s = chrono.ChVisualShapeCylinder(0.03, 0.4)
cyl_s.SetColor(chrono.ChColor(0.2, 0.2, 0.6))
slider.AddVisualShape(cyl_s, chrono.ChFramed(chrono.VNULL, chrono.QuatFromAngleX(chrono.CH_C_PI_2)))

## Connecting rod
rod = chrono.ChBody()
system.AddBody(rod)
rod.SetIdentifier(3)
rod.SetName("rod")
rod.SetMass(0.5)
rod.SetMass(0.5)
rod.SetInertiaXX(chrono.ChVector3d(0.005, 0.1, 0.1))
rod.SetPos(chrono.ChVector3d(0, 0, 0))
rod.SetRot(chrono.ChQuaterniond(1, 0, 0, 0))

box_r = chrono.ChVisualShapeBox(4, 0.1, 0.1)
rod.AddVisualShape(box_r)

cyl_r = chrono.ChVisualShapeCylinder(0.03, 0.4)
cyl_r.SetColor(chrono.ChColor(0.2, 0.6, 0.2))
rod.AddVisualShape(cyl_r, chrono.ChFramed(chrono.ChVector3d(2, 0, 0), chrono.QuatFromAngleX(chrono.CH_C_PI_2)));

  #### -------------------------------------------------------------------------
  #### EXERCISE 2.1
  #### Enable contact on the slider body and specify contact geometry
  #### The contact shape attached to the slider body should be a box with the
  #### same dimensions as the visualization asset, centered at the body origin.
  #### Use a coefficient of friction of 0.4.
  #### -------------------------------------------------------------------------
  
  ## TO DO


  #### -------------------------------------------------------------------------
  #### EXERCISE 2.2
  #### Create a new body, with a spherical shape (radius 0.2), used both as
  #### visualization asset and contact shape (mu = 0.4). This body should have:
  ####    mass: 1
  ####    moments of inertia: I_xx = I_yy = I_zz = 0.02
  ####    initial location: (5.5, 0, 0)
  #### -------------------------------------------------------------------------

  ## TO DO


## 3. Create joint constraints.
##    All joint frames are specified in the global frame.

## Define two quaternions representing:
## - a rotation of -90 degrees around x (z2y)
## - a rotation of +90 degrees around y (z2x)
z2y = chrono.ChQuaterniond()
z2x = chrono.ChQuaterniond()
z2y.SetFromAngleX(-chrono.CH_C_PI / 2)
z2x.SetFromAngleY(chrono.CH_C_PI / 2)

## Create a ChFunction object that always returns the constant value PI/2.
fun = chrono.ChFunctionConst()
fun.Set_yconst(chrono.CH_C_PI)

## Motor between ground and crank.
## Note that this also acts as a revolute joint (i.e. it enforces the same
## kinematic constraints as a revolute joint).  As before, we apply the 'z2y'
## rotation to align the rotation axis with the Y axis of the global frame.
engine_ground_crank = chrono.ChLinkMotorRotationSpeed()
engine_ground_crank.SetName("engine_ground_crank")
engine_ground_crank.Initialize(ground, crank, chrono.ChFramed(chrono.ChVector3d(0, 0, 0), z2y))
engine_ground_crank.SetSpeedFunction(fun)
system.AddLink(engine_ground_crank)

## Prismatic joint between ground and slider.
## The translational axis of a prismatic joint is along the Z axis of the
## specified joint coordinate system.  Here, we apply the 'z2x' rotation to
## align it with the X axis of the global reference frame.
prismatic_ground_slider = chrono.ChLinkLockPrismatic()
prismatic_ground_slider.SetName("prismatic_ground_slider")
prismatic_ground_slider.Initialize(ground, slider, chrono.ChFramed(chrono.ChVector3d(2, 0, 0), z2x))
system.AddLink(prismatic_ground_slider)

## Spherical joint between crank and rod
spherical_crank_rod = chrono.ChLinkLockSpherical()
spherical_crank_rod.SetName("spherical_crank_rod")
spherical_crank_rod.Initialize(crank, rod, chrono.ChFramed(chrono.ChVector3d(-2, 0, 0), chrono.QUNIT))
system.AddLink(spherical_crank_rod)

## Universal joint between rod and slider.
## The "cross" of a universal joint is defined using the X and Y axes of the
## specified joint coordinate frame. Here, we apply the 'z2x' rotation so that
## the cross is aligned with the Z and Y axes of the global reference frame.
universal_rod_slider = chrono.ChLinkUniversal()
universal_rod_slider.SetName("universal_rod_slider")
universal_rod_slider.Initialize(rod, slider, chrono.ChFramed(chrono.ChVector3d(2, 0, 0), z2x))
system.AddLink(universal_rod_slider)

  #### -------------------------------------------------------------------------
  #### EXERCISE 2.3
  #### Add a prismatic joint between ground and ball to constrain the ball's
  #### motion to the global X axis.
  #### -------------------------------------------------------------------------

  ## TODO


  #### -------------------------------------------------------------------------
  #### EXERCISE 2.4
  #### Add a spring-damper (ChLinkTSDA) between ground and the ball.
  #### This element should connect the center of the ball with the global point
  #### (6.5, 0, 0).  Set a spring constant of 50 and a spring free length of 1.
  #### Set a damping coefficient of 5.
  #### -------------------------------------------------------------------------

  ## TODO


## 4. Write the system hierarchy to the console (default log output destination)
####system.ShowHierarchy(chrono.GetLog())

## 5. Prepare visualization with Irrlicht
##    Note that Irrlicht uses left-handed frames with Y up.

## Create the Irrlicht application and set-up the camera.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Slider-Crank Demo 2')
vis.SetCameraVertical(chrono.CameraVerticalDir_Z)
vis.Initialize()
vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
vis.AddSkyBox()
vis.AddCamera(chrono.ChVector3d(2, -5, 0), chrono.ChVector3d(2, 0, 0))
vis.AddTypicalLights()

## 6. Perform the simulation.

## Specify the step-size.
step_size = 0.01
realtime_timer = chrono.ChRealtimeStepTimer()

while (vis.Run()):
    vis.BeginScene() 

    # Render Chrono item assets
    vis.Render()

    ## Draw an XZ grid at the global origin to add in visualization
    chronoirr.drawGrid(
        vis, 1, 1, 20, 20,
        chrono.ChCoordsysd(chrono.ChVector3d(0, 0, 0), chrono.QuatFromAngleX(chrono.CH_C_PI_2)),
        chrono.ChColor(0.4, 0.7, 0.4), True)
    chronoirr.drawAllCOGs(vis, 1)

    vis.EndScene()

    ## Advance simulation by one step
    system.DoStepDynamics(step_size)

    # Spin in place for real time to catch up
    realtime_timer.Spin(step_size)
