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
// Create a falling cable using FEA module (FEA tutorial n.1)
//
// This cable is made with N beam elements of ChElementANCFcable type. They are
// added to a ChMesh and then the first node is connected to the absolute
// reference using a constraint.
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
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono_fea/ChElementCableANCF.h"
#include "chrono_fea/ChBuilderBeam.h"
#include "chrono_fea/ChMesh.h"
#include "chrono_fea/ChVisualizationFEAmesh.h"
#include "chrono_fea/ChLinkPointFrame.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::fea;
using namespace irr;

int main(int argc, char* argv[]) {

    // 0. Set the path to the Chrono data folder

    SetChronoDataPath(CHRONO_DATA_DIR);

    // 1. Create the physical system that will handle all finite elements and constraints.

    //    Specify the gravitational acceleration vector, consistent with the
    //    global reference frame having Y up (ISO system).
    ChSystem system;
    system.Set_G_acc(ChVector<>(0, -9.81, 0));


    // 2. Create the mesh that will contain the finite elements, and add it to the system

    auto mesh = std::make_shared<ChMesh>();

    system.Add(mesh);


    // 3. Create a material for the beam finite elements.

    //    Note that each FEA element type requires some corresponding
    //    type of material. Here we will use ChElementCableANCF elements:
    //    they use a material of type ChBeamSectionCable, so let's do

    auto beam_material = std::make_shared<ChBeamSectionCable>();
    beam_material->SetDiameter(0.01);
    beam_material->SetYoungModulus(0.01e9);
    beam_material->SetBeamRaleyghDamping(0.01);


    // 4. Create the nodes

    //    - We use a simple for() loop to create nodes along the cable.
    //    - Nodes for ChElementCableANCF must be of ChNodeFEAxyzD class;
    //      i.e. each node has 6 coordinates: {position, direction}, where
    //      direction is the tangent to the cable.
    //    - Each node must be added to the mesh, ex.  mesh.Add(my_node)
    //    - To make things easier in the following, we store node pointers
    //      into an optional 'beam_nodes' array, i.e. a std::vector<>, later we
    //      can use such array for easy creation of elements between the nodes.

    std::vector<std::shared_ptr<ChNodeFEAxyzD> > beam_nodes;

    double length = 1.2;  // beam length, in meters;
    int N_nodes = 16;
    for (int in = 0; in < N_nodes; ++in) {
        // i-th node position
        ChVector<> position(length * (in / double(N_nodes - 1)),  // node position, x
                            0.5,                                  // node position, y
                            0);                                   // node position, z

        // i-th node direction
        ChVector<> direction(1.0, 0, 0);

        // create the node
        auto node = std::make_shared<ChNodeFEAxyzD>(position, direction);

        // add it to mesh
        mesh->AddNode(node);

        // add it to the auxiliary beam_nodes
        beam_nodes.push_back(node);
    }


    // 5. Create the elements

    //    - We use a simple for() loop to create elements between the
    //      nodes that we already created.
    //    - Each element must be set with the ChBeamSectionCable material
    //      that we already created
    //    - Each element must be added to the mesh, ex.  mesh.Add(my_element)

    for (int ie = 0; ie < N_nodes - 1; ++ie) {
        // create the element
        auto element = std::make_shared<ChElementCableANCF>();

        // set the connected nodes (pick two consecutive nodes in our beam_nodes container)
        element->SetNodes(beam_nodes[ie], beam_nodes[ie + 1]);

        // set the material
        element->SetSection(beam_material);

        // add it to mesh
        mesh->AddElement(element);
    }


    // 6. Add constraints

    //    - Constraints can be applied to FEA nodes
    //    - For the ChNodeFEAxyzD there are specific constraints that
    //      can be used to connect them to a ChBody, namely
    //      ChLinkPointFrame and ChLinkDirFrame
    //    - To attach one end of the beam to the ground, we need a
    //      'truss' ChBody that is fixed.
    //    - Note. An alternative, only when the node must be fixed 
    //      to absolute reference, is not using constraints, and just
    //      use: beam_nodes[0]->SetFixed(true);  (but would fix also dir)

    auto truss = std::make_shared<ChBody>();
    truss->SetBodyFixed(true);
    system.Add(truss);

    // lock an end of the wire to the truss
    auto constraint_pos = std::make_shared<ChLinkPointFrame>();
    constraint_pos->Initialize(beam_nodes[0], truss);
    system.Add(constraint_pos);
    

    // ADD ALSO A CYLINDER CONNECTED TO THE END OF THE CABLE
    
    // create the cylinder
    auto cylinder = std::make_shared<ChBodyEasyCylinder>(
                0.02,  // radius
                0.1,   // height
                1000,  // density (used to auto-set inertia, mass)
                true,  // do collide 
                true); // do visualize
    
    // move cylinder to end of beam
    cylinder->SetPos( beam_nodes.back()->GetPos() + ChVector<>(0, -0.05, 0) );

    // add it to the system
    system.Add(cylinder);

    // lock an end of the wire to the cylinder
    auto constraint_cyl = std::make_shared<ChLinkPointFrame>();
    constraint_cyl->Initialize(beam_nodes.back(), cylinder);
    system.Add(constraint_cyl);


    // 7. Make the finite elements visible in the 3D view

    //   - FEA fisualization can be managed via an easy
    //     ChVisualizationFEAmesh helper class.
    //     (Alternatively you could bypass this and output .dat
    //     files at each step, ex. for VTK or Matalb postprocessing)
    //   - This will automatically update a triangle mesh (a ChTriangleMeshShape
    //     asset that is internally managed) by setting proper
    //     coordinates and vertex colours as in the FEA elements.
    //   - Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    //     postprocessor that can handle a coloured ChTriangleMeshShape).
    //   - Do not forget AddAsset() at the end!

    auto mvisualizebeamA = std::make_shared<ChVisualizationFEAmesh>(*(mesh.get()));
    mvisualizebeamA->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_ANCF_BEAM_AX);
    mvisualizebeamA->SetColorscaleMinMax(-0.005, 0.005);
    mvisualizebeamA->SetSmoothFaces(true);
    mvisualizebeamA->SetWireframe(false);
    mesh->AddAsset(mvisualizebeamA);

    auto mvisualizebeamC = std::make_shared<ChVisualizationFEAmesh>(*(mesh.get()));
    mvisualizebeamC->SetFEMglyphType(
        ChVisualizationFEAmesh::E_GLYPH_NODE_DOT_POS);  // E_GLYPH_NODE_CSYS for ChNodeFEAxyzrot
    mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizebeamC->SetSymbolsThickness(0.006);
    mvisualizebeamC->SetSymbolsScale(0.005);
    mvisualizebeamC->SetZbufferHide(false);
    mesh->AddAsset(mvisualizebeamC);


    // 8. Configure the solver and timestepper

    //    - the default SOLVER_SOR of Chrono is not able to manage stiffness matrices
    //      as required by FEA! we must switch to a different solver.
    //    - We pick the SOLVER_MINRES solver and we configure it.
    //    - Note that if you build the MKL module, you could use the more precise MKL solver.

    // Change solver
    system.SetSolverType(ChSystem::SOLVER_MINRES);
    system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    system.SetMaxItersSolverSpeed(200);
    system.SetMaxItersSolverStab(200);
    system.SetTolForce(1e-10);

    // Change integrator:
    system.SetIntegrationType(chrono::ChSystem::INT_EULER_IMPLICIT_LINEARIZED);  // default: fast, 1st order
    // system.SetIntegrationType(chrono::ChSystem::INT_HHT);  // precise, slower, might iterate each step


    // 9. Prepare visualization with Irrlicht
    //    Note that Irrlicht uses left-handed frames with Y up.

    // Create the Irrlicht application and set-up the camera.
    ChIrrApp* application = new ChIrrApp(&system,                            // pointer to the mechanical system
                                         L"FEA cable collide demo",          // title of the Irrlicht window
                                         core::dimension2d<u32>(1024, 768),  // window dimension (width x height)
                                         false,                              // use full screen?
                                         true,                               // enable stencil shadows?
                                         true);                              // enable antialiasing?

    application->AddTypicalLogo();
    application->AddTypicalSky();
    application->AddTypicalLights();
    application->AddTypicalCamera(core::vector3df(0.1, 0.2, -2),  // camera location
                                  core::vector3df(0, 0, 0));      // "look at" location

    // Let the Irrlicht application convert the visualization assets.
    application->AssetBindAll();
    application->AssetUpdateAll();


    // 10. Perform the simulation.

    // Specify the step-size.
    application->SetTimestep(0.01);
    application->SetTryRealtime(true);

    // Mark completion of system construction
    system.SetupInitial();

    while (application->GetDevice()->run()) {
        // Initialize the graphical scene.
        application->BeginScene();

        // Render all visualization objects.
        application->DrawAll();

        // Draw an XZ grid at the global origin to add in visualization.
        ChIrrTools::drawGrid(application->GetVideoDriver(), 0.1, 0.1, 20, 20,
                             ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
                             video::SColor(255, 80, 100, 100), true);

        // Advance simulation by one step.
        application->DoStep();

        // Finalize the graphical scene.
        application->EndScene();
    }

    return 0;
}
