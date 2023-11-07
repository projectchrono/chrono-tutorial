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
## Slider-crank Chrono tutorial (model 0)
##
## This model is a 2-body slider-crank consisting of crank and slider bodies.
## The crank is connected to ground with a revolute joint and the slider is
## connected to ground through a prismatic joint.  A distance constraint models
## a massless link between the crank and the slider. 
##
## The mechanism moves under the action of gravity alone, acting in the negative
## Z direction.
##
## The simulation is animated with Irrlicht.
##
## =============================================================================

import pychrono as chrono
from pychrono import irrlicht as chronoirr

## 0. Set the path to the Chrono data folder
chrono.SetChronoDataPath('C:/codes/Chrono/Chrono_Source/data/')

## 1. Create the physical system that will handle all bodies and constraints.

##    Specify the gravitational acceleration vector, consistent with the
##    global reference frame having Z up.
system = chrono.ChSystemNSC()
system.Set_G_acc(chrono.ChVectorD(0, 0, -9.81))

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

cyl_g = chrono.ChVisualShapeCylinder()
cyl_g.GetCylinderGeometry().p1 = chrono.ChVectorD(0, 0.2, 0)
cyl_g.GetCylinderGeometry().p2 = chrono.ChVectorD(0, -0.2, 0)
cyl_g.GetCylinderGeometry().rad = 0.03
ground.AddAsset(cyl_g)

col_g = chrono.ChColorAsset()
col_g.SetColor(chrono.ChColor(0.6, 0.6, 0.2))
ground.AddAsset(col_g)

## Crank
crank = chrono.ChBody()
system.AddBody(crank)
crank.SetIdentifier(1)
crank.SetName("crank")
crank.SetMass(1.0)
crank.SetInertiaXX(chrono.ChVectorD(0.005, 0.1, 0.1))
crank.SetPos(chrono.ChVectorD(-1, 0, 0))
crank.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))

box_c = chrono.ChVisualShapeBox()
box_c.GetBoxGeometry().Size = chrono.ChVectorD(0.95, 0.05, 0.05)
crank.AddAsset(box_c)

cyl_c = chrono.ChVisualShapeCylinder()
cyl_c.GetCylinderGeometry().p1 = chrono.ChVectorD(1, 0.1, 0)
cyl_c.GetCylinderGeometry().p2 = chrono.ChVectorD(1, -0.1, 0)
cyl_c.GetCylinderGeometry().rad = 0.05
crank.AddAsset(cyl_c)

sph_c = chrono.ChVisualShapeSphere()
sph_c.GetSphereGeometry().center = chrono.ChVectorD(-1, 0, 0)
sph_c.GetSphereGeometry().rad = 0.05
crank.AddAsset(sph_c)

col_c = chrono.ChColorAsset()
col_c.SetColor(chrono.ChColor(0.6, 0.2, 0.2))
crank.AddAsset(col_c)

## Slider
slider = chrono.ChBody()
system.AddBody(slider)
slider.SetIdentifier(2)
slider.SetName("slider")
slider.SetMass(1.0)
slider.SetInertiaXX(chrono.ChVectorD(0.05, 0.05, 0.05))
slider.SetPos(chrono.ChVectorD(2, 0, 0))
slider.SetRot(chrono.ChQuaternionD(1, 0, 0, 0))

box_s = chrono.ChVisualShapeBox()
box_s.GetBoxGeometry().Size = chrono.ChVectorD(0.2, 0.1, 0.1)
slider.AddAsset(box_s)

col_s = chrono.ChColorAsset()
col_s.SetColor(chrono.ChColor(0.2, 0.2, 0.6))
slider.AddAsset(col_s)

## 3. Create joint constraints.
##    All joint frames are specified in the global frame.

## Define two quaternions representing:
## - a rotation of -90 degrees around x (z2y)
## - a rotation of +90 degrees around y (z2x)
z2y = chrono.ChQuaternionD() 
z2x = chrono.ChQuaternionD()
z2y.Q_from_AngAxis(-chrono.CH_C_PI / 2, chrono.ChVectorD(1, 0, 0))
z2x.Q_from_AngAxis(chrono.CH_C_PI / 2, chrono.ChVectorD(0, 1, 0))

## Revolute joint between ground and crank.
## The rotational axis of a revolute joint is along the Z axis of the
## specified joint coordinate frame.  Here, we apply the 'z2y' rotation to
## align it with the Y axis of the global reference frame.
revolute_ground_crank = chrono.ChLinkLockRevolute()
revolute_ground_crank.SetName("revolute_ground_crank")
revolute_ground_crank.Initialize(ground, crank, chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), z2y))
system.AddLink(revolute_ground_crank)

## Prismatic joint between ground and slider.
## The translational axis of a prismatic joint is along the Z axis of the
## specified joint coordinate system.  Here, we apply the 'z2x' rotation to
## align it with the X axis of the global reference frame.
prismatic_ground_slider = chrono.ChLinkLockPrismatic()
prismatic_ground_slider.SetName("prismatic_ground_slider")
prismatic_ground_slider.Initialize(ground, slider, chrono.ChCoordsysD(chrono.ChVectorD(2, 0, 0), z2x))
system.AddLink(prismatic_ground_slider)

## Distance constraint between crank and slider.
## We provide the points on the two bodies in the global reference frame.
## By default the imposed distance is calculated automatically as the distance
## between these two points in the initial configuration.
dist_crank_slider = chrono.ChLinkDistance()
dist_crank_slider.SetName("dist_crank_slider")
dist_crank_slider.Initialize(crank, slider, False, chrono.ChVectorD(-2, 0, 0), chrono.ChVectorD(2, 0, 0))
system.AddLink(dist_crank_slider)

## 4. Write the system hierarchy to the console (default log output destination)
system.ShowHierarchy(chrono.GetLog())

## 5. Prepare visualization with Irrlicht
##    Note that Irrlicht uses left-handed frames with Y up.

## Create the Irrlicht application and set-up the camera.
application = chronoirr.ChIrrApp(
        system,                               ## pointer to the mechanical system
        "Slider-Crank Demo 0",                ## title of the Irrlicht window
        chronoirr.dimension2du(800, 600),     ## window dimension (width x height)
        chronoirr.VerticalDir_Z)              ## up direction
application.AddLogo()
application.AddTypicalLights()
application.AddCamera(
        chronoirr.vector3df(2, -5, 0),        ## camera location
        chronoirr.vector3df(2, 0, 0))         ## "look at" location

## Let the Irrlicht application convert the visualization assets.
application.AssetBindAll()
application.AssetUpdateAll()
## 6. Perform the simulation.
## Specify the step-size.
application.SetTimestep(0.01)
application.SetTryRealtime(True)

while (application.GetDevice().run()):
    ## Initialize the graphical scene.
    application.BeginScene(True, True, chronoirr.SColor(255, 225, 225, 225))
    
    ## Render all visualization objects.
    application.DrawAll()

    ## Render the distance constraint.
    chronoirr.drawSegment(
        application.GetVideoDriver(),
        dist_crank_slider.GetEndPoint1Abs(),
        dist_crank_slider.GetEndPoint2Abs(),
        chronoirr.SColor(255, 200, 20, 0), True)

    ## Draw an XZ grid at the global origin to add in visualization.
    chronoirr.drawGrid(
        application.GetVideoDriver(), 1, 1, 20, 20,
        chrono.ChCoordsysD(chrono.ChVectorD(0, 0, 0), chrono.Q_from_AngX(chrono.CH_C_PI_2)),
        chronoirr.SColor(255, 80, 100, 100), True)
    chronoirr.drawAllCOGs(system, application.GetVideoDriver(), 1)

    ## Advance simulation by one step.
    application.DoStep()

    ## Finalize the graphical scene.
    application.EndScene()
