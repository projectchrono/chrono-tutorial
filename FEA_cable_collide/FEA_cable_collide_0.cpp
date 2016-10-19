// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Alessandro Tasora
// =============================================================================
//
// Create a falling cable using FEA module (model 0)
//
// This model is made with N elements of ChElementANCFcable type. They are
// added to a ChMesh and then the first cable is connected to the absolute
// reference using a joint.
//
// The cable falls under the action of gravity alone, acting in the negative
// Y (up) direction.
//
// The simulation is animated with Irrlicht.
//
// =============================================================================

#include <stdio.h>
#include <cmath>

#include "chrono/physics/ChSystem.h"
#include "chrono_irrlicht/ChIrrApp.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace irr;


int main(int   argc,
         char* argv[])
{
  // 0. Set the path to the Chrono data folder

  SetChronoDataPath(CHRONO_DATA_DIR);


  // 1. Create the physical system that will handle all finite elements and constraints.

  //    Specify the gravitational acceleration vector, consistent with the
  //    global reference frame having Y up (ISO system).
  ChSystem system;
  system.Set_G_acc(ChVector<>(0, -9.81, 0));


  // 2. Create the mesh that will contain the finite elements

  //***TODO***


  // 5. Prepare visualization with Irrlicht
  //    Note that Irrlicht uses left-handed frames with Y up.

  // Create the Irrlicht application and set-up the camera.
  ChIrrApp * application = new ChIrrApp(
    &system,                               // pointer to the mechanical system
    L"FEA cable collide demo",             // title of the Irrlicht window
    core::dimension2d<u32>(800, 600),      // window dimension (width x height)
    false,                                 // use full screen?
    true,                                  // enable stencil shadows?
    true);                                 // enable antialiasing?

  application->AddTypicalLogo();
  application->AddTypicalSky();
  application->AddTypicalLights();
  application->AddTypicalCamera(
    core::vector3df(2, 5, -3),             // camera location
    core::vector3df(2, 0, 0));             // "look at" location

  // Let the Irrlicht application convert the visualization assets.
  application->AssetBindAll();
  application->AssetUpdateAll();


  // 6. Perform the simulation.

  // Specify the step-size.
  application->SetTimestep(0.01);
  application->SetTryRealtime(true);

  while (application->GetDevice()->run())
  {
    // Initialize the graphical scene.
    application->BeginScene();
    
    // Render all visualization objects.
    application->DrawAll();


    // Draw an XZ grid at the global origin to add in visualization.
    ChIrrTools::drawGrid(
      application->GetVideoDriver(), 1, 1, 20, 20,
      ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
      video::SColor(255, 80, 100, 100), true);

    // Advance simulation by one step.
    application->DoStep();

    // Finalize the graphical scene.
    application->EndScene();
  }

  return 0;
}


