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
// Apply loads to a falling and colliding cable using FEA module (FEA tutorial n.3)
//
// This model is made with N elements of ChElementBeamEuler type. They are
// added to a ChMesh and then the first cable is connected to the absolute
// reference using a joint.
//
// A simple ChContactSurfaceNodeCloud is used to provide collision against
// the floor.
//
// The ChLoad approach is used to apply loads along the beam.
//
// The cable falls under the action of gravity alone, acting in the negative
// Y (up) direction.
//
// The simulation is animated with Irrlicht.
//
// =============================================================================

#include <cstdio>
#include <cmath>

#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChLinkMate.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono_irrlicht/ChIrrApp.h"

#include "chrono/fea/ChElementBeamEuler.h"
#include "chrono/fea/ChBuilderBeam.h"
#include "chrono/fea/ChMesh.h"
#include "chrono/fea/ChVisualizationFEAmesh.h"
#include "chrono/fea/ChContactSurfaceMesh.h"
#include "chrono/fea/ChContactSurfaceNodeCloud.h"
#include "chrono/physics/ChLoadContainer.h"
#include "chrono/fea/ChLoadsBeam.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::fea;
using namespace irr;

int main(int argc, char* argv[]) {

    // 0. Set the path to the Chrono data folder

    SetChronoDataPath(CHRONO_DATA_DIR);

    // 1. Create the physical system that will handle all finite elements and constraints.

    //    NOTE that we need contact in FEA, so we use the ChSystemSMC, that uses SMC penalty in contacts
    ChSystemSMC system;
    system.Set_G_acc(ChVector<>(0, -9.81, 0));


    // 2. Create the mesh that will contain the finite elements, and add it to the system

    auto mesh = std::make_shared<ChMesh>();

    system.Add(mesh);


    // 3. Create a material for the beam finite elements.

    //    Note that each FEA element type requires some corresponding
    //    type of material. ChElemetBeamEuler require a ChBeamSectionAdvanced material.

    auto beam_material = std::make_shared<ChBeamSectionAdvanced>();
	beam_material->SetAsRectangularSection(0.012, 0.025);
	beam_material->SetYoungModulus (0.01e9);
	beam_material->SetGshearModulus(0.01e9 * 0.3);
	beam_material->SetBeamRaleyghDamping(0.01);


    // 4. Create the nodes

    //    - We use a simple for() loop to create nodes along the cable.
    //    - Nodes for ChElemetBeamEuler must be of ChNodeFEAxyzrot class;
    //      i.e. each node has coordinates of type: {position, rotation},
    //      where X axis of rotated system is the direction of the beam, 
    //      Y and Z are the section plane.

    std::vector<std::shared_ptr<ChNodeFEAxyzrot> > beam_nodes;

    double length = 1.2;  // beam length, in meters;
    int N_nodes = 16;
    for (int in = 0; in < N_nodes; ++in) {
        // i-th node position
        ChVector<> position(length * (in / double(N_nodes - 1)),  // node position, x
                            0.5,                                  // node position, y
                            0);                                   // node position, z

        // create the node
        auto node = std::make_shared<ChNodeFEAxyzrot>( ChFrame<>(position) );

        // add it to mesh
        mesh->AddNode(node);

        // add it to the auxiliary beam_nodes
        beam_nodes.push_back(node);
    }


    // 5. Create the elements

    std::vector<std::shared_ptr<ChElementBeamEuler> > beam_elements; // only for easy access later, when adding loads

    for (int ie = 0; ie < N_nodes - 1; ++ie) {
        // create the element
        auto element = std::make_shared<ChElementBeamEuler>();

        // set the connected nodes (pick two consecutive nodes in our beam_nodes container)
        element->SetNodes(beam_nodes[ie], beam_nodes[ie + 1]);

        // set the material
        element->SetSection(beam_material);

        // add it to mesh
        mesh->AddElement(element);

        // add it to the auxiliary beam_elements
        beam_elements.push_back(element);
    }


    // 6. Add constraints

    //    - Constraints can be applied to FEA nodes
    //    - For the ChNodeFEAxyzrot one can use all constraints
    //      of the ChMate class

    auto truss = std::make_shared<ChBody>();
    truss->SetBodyFixed(true);
    system.Add(truss);

    // lock an end of the wire to the truss
    auto constraint_pos = std::make_shared<ChLinkMateSpherical>();
    constraint_pos->Initialize(
        beam_nodes[0],  // node to constraint
        truss,          // body to constraint
        false,          // false: next 2 pos are in absolute coords, true: in relative coords
        beam_nodes[0]->GetPos(), // sphere ball pos 
        beam_nodes[0]->GetPos()  // sphere cavity pos
        );
    system.Add(constraint_pos);


    // 7. Add a collision mesh to the skin of the finite element mesh

    //    - Create a ChMaterialSurfaceSMC, it must be assigned to FEA 
    //      meshes and rigid bodies. The ChSystemSMC requires it!
    //    - Create a ChContactSurfaceNodeCloud and add to the FEA mesh.
    //      This is the easiest representation of a FEA contact surface: it
    //      simply creates contact spheres per each node. So, no edge-edge cases
    //      can be detected between elements though, but it is enough for
    //      dense finite elements meshes that collide with large objects.

    // Create a surface material to be shared with some objects
    auto mysurfmaterial = std::make_shared<ChMaterialSurfaceSMC>();
    mysurfmaterial->SetYoungModulus(6e4);
    mysurfmaterial->SetFriction(0.3f);
    mysurfmaterial->SetRestitution(0.2f);
    mysurfmaterial->SetAdhesion(0); 

    // Create the contact surface and add to the mesh
    auto mcontactcloud = std::make_shared<ChContactSurfaceNodeCloud>();
    mesh->AddContactSurface(mcontactcloud);
    
    // Must use this to 'populate' the contact surface use larger point size to match beam section radius
    mcontactcloud->AddAllNodes(0.01); 

    // Use our SMC surface material properties 
    mcontactcloud->SetMaterialSurface(mysurfmaterial);


    // 8. Create a collision plane, as a huge box

    auto floor = std::make_shared<ChBodyEasyBox>(
          4, 0.2, 4,  // x,y,z size
          1000,       // density
          true,       // visible
          true        // collide
        );

    system.Add(floor);

    floor->SetBodyFixed(true);
    floor->SetPos( ChVector<>(0,-0.1,0) );

    // Use our SMC surface material properties 
    floor->SetMaterialSurface(mysurfmaterial);


    // 9. Apply some loads to the beam. 
    //
    //   - A ChLoadContainer must be added to the system
    //   - You must create ChLoad objects to add to the ChLoadContainer
    //   - Option 1: use ready-to-use loads like those in chrono_fea/ChLoadsBeam.h
    //   - Option 2.a: implement your own ChLoad-inherited class
    //   - Option 2.b: implement your own ChLoader-inherited class and use in a ChLoad

    // Create the load container and add it to your ChSystem 
    auto loadcontainer = std::make_shared<ChLoadContainer>();
    system.Add(loadcontainer);

    // Example: Add a vertical load to the end point of the last beam element:
    auto mwrench = std::make_shared<ChLoadBeamWrench>(beam_elements.back());
    mwrench->loader.SetApplication(1.0); // in -1..+1 range, -1: end A, 0: mid, +1: end B
    mwrench->loader.SetForce(ChVector<>(0,-0.2,0));
    loadcontainer->Add(mwrench);  // do not forget to add the load to the load container.

    //// -------------------------------------------------------------------------
    //// EXERCISE 1
    ////
    //// Use a distributed vertical load, instead of a point load, applied
    //// as a constant value along the last element.
    //// Hint: it is not much different than the example few rows above
    //// Hint: use the ChLoadBeamWrenchDistributed class
    ////
    //// -------------------------------------------------------------------------


    // Example: Create a custom distributed load: a horizontal time-dependant sinusoidal load.
    // We will use a ChLoader class. A ChLoader is an object that is used by a ChLoad to apply 
    // loads to a ChLoadable object: this is useful especially when the load is distributed, because
    // the ChLoad will automatically use ChLoader many times along the integration points.
    // There are some stubs in the ChLoaderU.h ChLoaderUV.h ChLoaderUVW.h , inherit from them.
    // For example, let's make a distributed time-dependant load. A load on the beam is a 
    // wrench, i.e. force+load per unit lenght aplied at a certain abscyssa U, that is a six-dimensional load. 

    class MyLoaderHorizontalSine : public ChLoaderUdistributed {
    public:
            // Useful: a constructor that also sets ChLoadable    
            MyLoaderHorizontalSine(std::shared_ptr<ChLoadableU> mloadable) 
                :  ChLoaderUdistributed(mloadable) {}

            // Compute F=F(u)
            // This is the function that YOU MUST implement. It should return the
            // load at U, per unit lenght. For Eulero beams, loads are expected as 6-rows vectors, i.e.
            // a wrench: forceX, forceY, forceZ, torqueX, torqueY, torqueZ. 
            virtual void ComputeF(const double U,     ///< parametric coordinate in line
                          ChVectorDynamic<>& F,       ///< Result F vector here, size must be = n.field coords.of loadable
                          ChVectorDynamic<>* state_x, ///< if != 0, update state (pos. part) to this, then evaluate F
                          ChVectorDynamic<>* state_w  ///< if != 0, update state (speed part) to this, then evaluate F
                          ) override {
                assert(auxsystem);
                double T = auxsystem->GetChTime();
                double freq = 4;
                double Fmax = 20; // btw, Fmax is in Newton per unit length, this is a distributed load
                double Fz = Fmax*sin(T*freq); // compute the time-dependant load
                // Return load in the F wrench: {forceX, forceY, forceZ, torqueX, torqueY, torqueZ}.
                F.PasteVector( ChVector<>(0, 0, Fz), 0,0); // load, force part; 
                F.PasteVector( ChVector<>(0,0,0)    ,3,0); // load, torque part; 
            }

            // Needed because inheriting ChLoaderUdistributed. Use 1 because linear load fx.
            virtual int GetIntegrationPointsU() override {return 1;}

    public:
            // add auxiliary data to the class, if you need to access it during ComputeF().
            ChSystem* auxsystem;
    };

    // Create a custom load that uses the custom loader above.
    // The ChLoad is a 'manager' for your ChLoader.
    // It is created using templates, that is instancing a ChLoad<my_loader_class>()

    std::shared_ptr< ChLoad<MyLoaderHorizontalSine> > mloadsine (new ChLoad<MyLoaderHorizontalSine>(beam_elements.back()) );
    mloadsine->loader.auxsystem = &system; // initialize auxiliary data of the loader, if needed
    loadcontainer->Add(mloadsine);  // do not forget to add the load to the load container.


    
    //// -------------------------------------------------------------------------
    //// EXERCISE 2
    ////
    //// a) instead of a sine wave, try using a square wave
    //// b) instead of a custom distributed load, use a custom point load (atomic load)
    ////
    //// Hint: it is not much different than the example few rows above.
    //// Hint: Instead of inheriting from ChLoaderUdistributed, inherit 
    ////       from ChLoaderUatomic class.
    //// Hint: to compute a square wave, look at the sign of a sine wave.
    //// Hint: use a small value for the amplitude of the square wave, ex 0.8 Newtons,
    ////       otherwise the beam might wobble too much
    //// -------------------------------------------------------------------------





    // 10. Make the finite elements visible in the 3D view

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
    mvisualizebeamC->SetFEMglyphType(ChVisualizationFEAmesh::E_GLYPH_NODE_CSYS); 
    mvisualizebeamC->SetFEMdataType(ChVisualizationFEAmesh::E_PLOT_NONE);
    mvisualizebeamC->SetSymbolsThickness(0.006);
    mvisualizebeamC->SetSymbolsScale(0.005);
    mvisualizebeamC->SetZbufferHide(false);
    mesh->AddAsset(mvisualizebeamC);


    // 10. Configure the solver and timestepper

    //    - the default SOLVER_SOR of Chrono is not able to manage stiffness matrices
    //      as required by FEA! we must switch to a different solver.
    //    - We pick the SOLVER_MINRES solver and we configure it.
    //    - Note that if you build the MKL module, you could use the more precise MKL solver.

    // Change solver
    system.SetSolverType(ChSolver::Type::MINRES);
    system.SetSolverWarmStarting(true);  // this helps a lot to speedup convergence in this class of problems
    system.SetMaxItersSolverSpeed(200);
    system.SetMaxItersSolverStab(200);
    system.SetTolForce(1e-10);

    // Change integrator:
    // system.SetTimestepperType(ChTimestepper::Type::EULER_IMPLICIT_LINEARIZED);  // default: fast, 1st order
    // system.SetTimestepperType(ChTimestepper::Type::HHT);  // precise, slower, might iterate each step


    // 11. Prepare visualization with Irrlicht
    //    Note that Irrlicht uses left-handed frames with Y up.

    // Create the Irrlicht application and set-up the camera.
    ChIrrApp application(&system,                            // pointer to the mechanical system
                                         L"FEA cable collide demo",          // title of the Irrlicht window
                                         core::dimension2d<u32>(1024, 768),  // window dimension (width x height)
                                         false,                              // use full screen?
                                         true,                               // enable stencil shadows?
                                         true);                              // enable antialiasing?

    application.AddTypicalLogo();
    application.AddTypicalSky();
    application.AddTypicalLights();
    application.AddTypicalCamera(core::vector3df(0.1f, 0.2f, -2.0f),  // camera location
                                  core::vector3df(0.0f, 0.0f, 0.0f));  // "look at" location

    // Enable drawing of contacts
    application.SetContactsDrawMode(ChIrrTools::CONTACT_FORCES);
    application.SetSymbolscale(0.1);

    // Let the Irrlicht application convert the visualization assets.
    application.AssetBindAll();
    application.AssetUpdateAll();


    // 12. Perform the simulation.

    // Specify the step-size.
    application.SetTimestep(0.001);
    application.SetTryRealtime(false);

    // Mark completion of system construction
    system.SetupInitial();

    while (application.GetDevice()->run()) {
        // Initialize the graphical scene.
        application.BeginScene();

        // Render all visualization objects.
        application.DrawAll();

        // Draw an XZ grid at the global origin to add in visualization.
        ChIrrTools::drawGrid(application.GetVideoDriver(), 0.1, 0.1, 20, 20,
                             ChCoordsys<>(ChVector<>(0, 0, 0), Q_from_AngX(CH_C_PI_2)),
                             video::SColor(255, 80, 100, 100), true);

        // Advance simulation by one step.
        application.DoStep();

        // Finalize the graphical scene.
        application.EndScene();
    }

    return 0;
}
