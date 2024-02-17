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
## Slider-crank Chrono tutorial (model 1)
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

  #### -------------------------------------------------------------------------
  #### EXERCISE 1.1
  #### Create a connecting rod body to replace the distance constraint.
  #### This body should have:
  ####    mass: 0.5
  ####    moments of inertia:  I_xx = 0.005, I_yy = 0.1, I_zz = 0.1
  ####    visualization: a green box with width and height 0.1
  #### -------------------------------------------------------------------------




## 3. Create joint constraints.
##    All joint frames are specified in the global frame.

## Define two quaternions representing:
## - a rotation of -90 degrees around x (z2y)
## - a rotation of +90 degrees around y (z2x)
z2y = chrono.ChQuaterniond()
z2x = chrono.ChQuaterniond()
z2y.SetFromAngleX(-chrono.CH_C_PI / 2)
z2x.SetFromAngleY(chrono.CH_C_PI / 2)

  #### -------------------------------------------------------------------------
  #### EXERCISE 1.2
  #### Replace the revolute joint between ground and crank with a
  #### ChLinkMotorRotationSpeed element and enforce constant angular speed of
  #### 90 degrees/s.
  #### -------------------------------------------------------------------------




## Prismatic joint between ground and slider.
## The translational axis of a prismatic joint is along the Z axis of the
## specified joint coordinate system.  Here, we apply the 'z2x' rotation to
## align it with the X axis of the global reference frame.
prismatic_ground_slider = chrono.ChLinkLockPrismatic()
prismatic_ground_slider.SetName("prismatic_ground_slider")
prismatic_ground_slider.Initialize(ground, slider, chrono.ChCoordsysd(chrono.ChVector3d(2, 0, 0), z2x))
system.AddLink(prismatic_ground_slider)

  #### -------------------------------------------------------------------------
  #### EXERCISE 1.3
  #### Replace the distance constraint with joints connecting the rod to the
  #### crank (use ChLinkLockSpherical) and to the slider (ChLinkUniversal). The
  #### universal joint's cross should be aligned with the Z and Y global axes.
  #### -------------------------------------------------------------------------




## 4. Write the system hierarchy to the console (default log output destination)
####system.ShowHierarchy(chrono.GetLog())


## 5. Prepare visualization with Irrlicht
##    Note that Irrlicht uses left-handed frames with Y up.

## Create the Irrlicht application and set-up the camera.
vis = chronoirr.ChVisualSystemIrrlicht()
vis.AttachSystem(system)
vis.SetWindowSize(1024,768)
vis.SetWindowTitle('Slider-Crank Demo 1')
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
