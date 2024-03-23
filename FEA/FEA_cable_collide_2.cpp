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
// Create a falling and colliding cable using FEA module (FEA tutorial n.2)
//
// This model is made with N elements of ChElementBeamEuler type. They are
// added to a ChMesh and then the first cable is connected to the absolute
// reference using a joint.
//
// A simple ChContactSurfaceNodeCloud is used to provide collision against
// the floor.
//
// The cable falls under the action of gravity alone, acting in the negative
// Y (up) direction.
//
// The simulation is animated with Irrlicht.
//
// =============================================================================

#include <cmath>
#include <cstdio>

#include "chrono/physics/ChBodyEasy.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/solver/ChIterativeSolverLS.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/assets/ChVisualShapeFEA.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::fea;
using namespace irr;

int main(int argc, char* argv[]) {
    // 0. Set the path to the Chrono data folder

    SetChronoDataPath(CHRONO_DATA_DIR);

    // 1. Create the physical system that will handle all finite elements and constraints.

    //    NOTE that we need contact in FEA, so we use the ChSystemSMC, that uses SMC  penalty in contacts
    ChSystemSMC system;

    system.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // 2. Create the mesh that will contain the finite elements, and add it to the system

    auto mesh = chrono_types::make_shared<ChMesh>();

    system.Add(mesh);

    //// -------------------------------------------------------------------------
    //// EXERCISE 1
    ////
    //// Use Euler-Bernoulli beams to make a hanging cable, exactly as in
    //// the FEA_cable_collide_1.cpp, but with ChElementBeamEuler elements
    //// instead of ChElementCableANCF.
    //// The ChElementBeamEuler beams are more sophisticated as they can also
    //// simulate torsion and shear, and off-center shear effects.
    //// Just use the same for() loops of the previous demo, but use these hints:
    //// Hint: when creating the section material, in 3., remember that
    ////       ChElementBeamEuler needs a ChBeamSectionAdvanced material.
    //// Hint: when creating the nodes, in 4., the nodes
    ////       for ChElementBeamEuler must be of ChNodeFEAxyzrot class;
    ////       i.e. each node has coordinates of type: {position, rotation},
    ////       where X axis of rotated system is the direction of the beam,
    ////       Y and Z are the section plane.
    //// Hint: when creating the elements, in 5., use ChElemetBeamEuler
    //// Hint: when creating the truss-node constraint, in 6., use ChLinkMateSpherical
    ////
    //// -------------------------------------------------------------------------

    // 3. Create a material for the beam finite elements.
    // TO DO ...

    // 4. Create the nodes
    // TO DO ....

    // 5. Create the elements
    // TO DO ....

    // 6. Add constraints
    // TO DO ....

    // 7. Add a collision mesh to the skin of the finite element mesh

    //    - Create a ChContactMaterialSMC , it must be assigned to FEA
    //      meshes and rigid bodies. The ChSystemSMC requires it!
    //    - Create a ChContactSurfaceNodeCloud and add to the FEA mesh.
    //      This is the easiest representation of a FEA contact surface: it
    //      simply creates contact spheres per each node. So, no edge-edge cases
    //      can be detected between elements though, but it is enough for
    //      dense finite elements meshes that collide with large objects.

    // Create a surface material to be shared with some objects
    auto surfmaterial = chrono_types::make_shared<ChContactMaterialSMC>();
    surfmaterial->SetYoungModulus(6e4);
    surfmaterial->SetFriction(0.3f);
    surfmaterial->SetRestitution(0.2f);
    surfmaterial->SetAdhesion(0);

    // Create the contact surface and add to the mesh, using our SMC contact material
    auto contactcloud = chrono_types::make_shared<ChContactSurfaceNodeCloud>(surfmaterial);
    mesh->AddContactSurface(contactcloud);

    // Must use this to 'populate' the contact surface use larger point size to match beam section radius
    contactcloud->AddAllNodes(0.01);

    // 8. Create a collision plane, as a huge box

    auto floor = chrono_types::make_shared<ChBodyEasyBox>(4, 0.2, 4,    // x,y,z size
                                                          1000,         // density
                                                          true,         // visible
                                                          true,         // collide
                                                          surfmaterial  // contact material
    );

    system.Add(floor);

    floor->SetFixed(true);
    floor->SetPos(ChVector3d(0, -0.1, 0));

    // 9. Make the finite elements visible in the 3D view

    //   - FEA fisualization can be managed via an easy
    //     ChVisualizationFEAmesh helper class.
    //     (Alternatively you could bypass this and output .dat
    //     files at each step, ex. for VTK or Matalb postprocessing)
    //   - This will automatically update a triangle mesh (a ChVisualShapeTriangleMesh
    //     asset that is internally managed) by setting proper
    //     coordinates and vertex colours as in the FEA elements.
    //   - Such triangle mesh can be rendered by Irrlicht or POVray or whatever
    //     postprocessor that can handle a coloured ChVisualShapeTriangleMesh).
    //   - Do not forget AddAsset() at the end!

    auto visualizebeamA = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    visualizebeamA->SetFEMdataType(ChVisualShapeFEA::DataType::ANCF_BEAM_AX);
    visualizebeamA->SetColorscaleMinMax(-0.005, 0.005);
    visualizebeamA->SetSmoothFaces(true);
    visualizebeamA->SetWireframe(false);
    mesh->AddVisualShapeFEA(visualizebeamA);

    auto visualizebeamB = chrono_types::make_shared<ChVisualShapeFEA>(mesh);
    visualizebeamB->SetFEMglyphType(ChVisualShapeFEA::GlyphType::NODE_DOT_POS);
    visualizebeamB->SetFEMdataType(ChVisualShapeFEA::DataType::NONE);
    visualizebeamB->SetSymbolsThickness(0.006);
    visualizebeamB->SetSymbolsScale(0.005);
    visualizebeamB->SetZbufferHide(false);
    mesh->AddVisualShapeFEA(visualizebeamB);

    // 10. Configure the solver and timestepper

    //    - the default SOLVER_SOR of Chrono is not able to manage stiffness matrices
    //      as required by FEA! we must switch to a different solver.
    //    - We pick the SOLVER_MINRES solver and we configure it.
    //    - Note that if you build the MKL module, you could use the more precise MKL solver.

    // Change solver
    auto solver = chrono_types::make_shared<ChSolverMINRES>();
    solver->SetMaxIterations(200);
    solver->SetTolerance(1e-10);
    solver->EnableWarmStart(true);
    system.SetSolver(solver);

    // Change integrator:
    // system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // default: fast, 1st order
    // system.SetTimestepperType(ChTimestepper::Type::HHT);  // precise, slower, might iterate each step

    // 11. Prepare visualization with Irrlicht
    //    Note that Irrlicht uses left-handed frames with Y up.

    // Create the Irrlicht application and set-up the camera.
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(1024, 768);
    vis->SetWindowTitle("FEA cable collide demo");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddTypicalLights();
    vis->AddCamera(ChVector3d(0.1f, 0.2f, -2.0f));
    vis->EnableContactDrawing(ContactsDrawMode::CONTACT_FORCES);
    vis->SetSymbolScale(0.1);
    vis->AttachSystem(&system);

    // 12. Perform the simulation.

    while (vis->Run()) {
        // Initialize the graphical scene.
        vis->BeginScene();

        // Render all visualization objects.
        vis->Render();

        // Draw an XZ grid at the global origin to add in visualization.
        tools::drawGrid(vis.get(), 0.1, 0.1, 20, 20, ChCoordsys<>(ChVector3d(0, 0, 0), QuatFromAngleX(CH_PI_2)),
                        ChColor(0.3f, 0.4f, 0.4f), true);

        // Finalize the graphical scene.
        vis->EndScene();

        // Advance simulation by one step.
        system.DoStepDynamics(0.001);
    }

    return 0;
}
