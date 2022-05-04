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
// Author: Radu Serban
// =============================================================================
//
// Slider-crank Chrono tutorial (model 0)
//
// This model is a 2-body slider-crank consisting of crank and slider bodies.
// The crank is connected to ground with a revolute joint and the slider is
// connected to ground through a prismatic joint.  A distance constraint models
// a massless link between the crank and the slider.
//
// The mechanism moves under the action of gravity alone, acting in the negative
// Z direction.
//
// The simulation is animated with Irrlicht.
//
// =============================================================================

#include <cmath>
#include <cstdio>

#include "chrono/physics/ChSystemNSC.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono_irrlicht/ChVisualSystemIrrlicht.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace irr;

int main(int argc, char* argv[]) {
    // 0. Set the path to the Chrono data folder

    SetChronoDataPath(CHRONO_DATA_DIR);

    // 1. Create the physical system that will handle all bodies and constraints.

    //    Specify the gravitational acceleration vector, consistent with the
    //    global reference frame having Z up.
    ChSystemNSC system;
    system.Set_G_acc(ChVector<>(0, 0, -9.81));

    // 2. Create the rigid bodies of the slider-crank mechanical system.
    //    For each body, specify:
    //    - a unique identifier
    //    - mass and moments of inertia
    //    - position and orientation of the (centroidal) body frame
    //    - visualization assets (defined with respect to the body frame)

    // Ground
    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetIdentifier(-1);
    ground->SetName("ground");
    ground->SetBodyFixed(true);

    auto cyl_g = chrono_types::make_shared<ChCylinderShape>();
    cyl_g->GetCylinderGeometry().p1 = ChVector<>(0, 0.2, 0);
    cyl_g->GetCylinderGeometry().p2 = ChVector<>(0, -0.2, 0);
    cyl_g->GetCylinderGeometry().rad = 0.03;
    cyl_g->SetColor(ChColor(0.6f, 0.6f, 0.2f));
    ground->AddVisualShape(cyl_g);

    // Crank
    auto crank = chrono_types::make_shared<ChBody>();
    system.AddBody(crank);
    crank->SetIdentifier(1);
    crank->SetName("crank");
    crank->SetMass(1.0);
    crank->SetInertiaXX(ChVector<>(0.005, 0.1, 0.1));
    crank->SetPos(ChVector<>(-1, 0, 0));
    crank->SetRot(ChQuaternion<>(1, 0, 0, 0));

    auto box_c = chrono_types::make_shared<ChBoxShape>();
    box_c->GetBoxGeometry().Size = ChVector<>(0.95, 0.05, 0.05);
    box_c->SetColor(ChColor(0.6f, 0.2f, 0.2f));
    crank->AddVisualShape(box_c);

    auto cyl_c = chrono_types::make_shared<ChCylinderShape>();
    cyl_c->GetCylinderGeometry().p1 = ChVector<>(1, 0.1, 0);
    cyl_c->GetCylinderGeometry().p2 = ChVector<>(1, -0.1, 0);
    cyl_c->GetCylinderGeometry().rad = 0.05;
    cyl_c->SetColor(ChColor(0.6f, 0.2f, 0.2f));
    crank->AddVisualShape(cyl_c);

    auto sph_c = chrono_types::make_shared<ChSphereShape>();
    sph_c->GetSphereGeometry().rad = 0.05;
    sph_c->SetColor(ChColor(0.6f, 0.2f, 0.2f));
    crank->AddVisualShape(sph_c, ChFrame<>(ChVector<>(-1, 0, 0)));

    // Slider
    auto slider = chrono_types::make_shared<ChBody>();
    system.AddBody(slider);
    slider->SetIdentifier(2);
    slider->SetName("slider");
    slider->SetMass(1.0);
    slider->SetInertiaXX(ChVector<>(0.05, 0.05, 0.05));
    slider->SetPos(ChVector<>(2, 0, 0));
    slider->SetRot(ChQuaternion<>(1, 0, 0, 0));

    auto box_s = chrono_types::make_shared<ChBoxShape>();
    box_s->GetBoxGeometry().Size = ChVector<>(0.2, 0.1, 0.1);
    box_s->SetColor(ChColor(0.2f, 0.2f, 0.6f));
    slider->AddVisualShape(box_s);

    // 3. Create joint constraints.
    //    All joint frames are specified in the global frame.

    // Define two quaternions representing:
    // - a rotation of -90 degrees around x (z2y)
    // - a rotation of +90 degrees around y (z2x)
    ChQuaternion<> z2y;
    ChQuaternion<> z2x;
    z2y.Q_from_AngAxis(-CH_C_PI / 2, ChVector<>(1, 0, 0));
    z2x.Q_from_AngAxis(CH_C_PI / 2, ChVector<>(0, 1, 0));

    // Revolute joint between ground and crank.
    // The rotational axis of a revolute joint is along the Z axis of the
    // specified joint coordinate frame.  Here, we apply the 'z2y' rotation to
    // align it with the Y axis of the global reference frame.
    auto revolute_ground_crank = chrono_types::make_shared<ChLinkLockRevolute>();
    revolute_ground_crank->SetName("revolute_ground_crank");
    revolute_ground_crank->Initialize(ground, crank, ChCoordsys<>(ChVector<>(0, 0, 0), z2y));
    system.AddLink(revolute_ground_crank);

    // Prismatic joint between ground and slider.
    // The translational axis of a prismatic joint is along the Z axis of the
    // specified joint coordinate system.  Here, we apply the 'z2x' rotation to
    // align it with the X axis of the global reference frame.
    auto prismatic_ground_slider = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic_ground_slider->SetName("prismatic_ground_slider");
    prismatic_ground_slider->Initialize(ground, slider, ChCoordsys<>(ChVector<>(2, 0, 0), z2x));
    system.AddLink(prismatic_ground_slider);

    // Distance constraint between crank and slider.
    // We provide the points on the two bodies in the global reference frame.
    // By default the imposed distance is calculated automatically as the distance
    // between these two points in the initial configuration.
    auto dist_crank_slider = chrono_types::make_shared<ChLinkDistance>();
    dist_crank_slider->SetName("dist_crank_slider");
    dist_crank_slider->Initialize(crank, slider, false, ChVector<>(-2, 0, 0), ChVector<>(2, 0, 0));
    system.AddLink(dist_crank_slider);

    // 4. Write the system hierarchy to the console (default log output destination)
    system.ShowHierarchy(GetLog());

    // 5. Prepare visualization with Irrlicht
    //    Note that Irrlicht uses left-handed frames with Y up.

    // Create the Irrlicht application and set-up the camera.
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    system.SetVisualSystem(vis);
    vis->SetWindowSize(800, 600);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowTitle("Slider-Crank Demo 0");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector<>(2, -5, 0), ChVector<>(2, 0, 0));
    vis->AddTypicalLights();

    // 6. Perform the simulation.

    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        // Initialize the graphical scene.
        vis->BeginScene();

        // Render all visualization objects.
        vis->DrawAll();

        // Render the distance constraint.
        tools::drawSegment(vis.get(), dist_crank_slider->GetEndPoint1Abs(),
                           dist_crank_slider->GetEndPoint2Abs(), ChColor(0.8f, 0.2f, 0), true);

        // Draw an XZ grid at the global origin to add in visualization.
        tools::drawGrid(vis.get(), 1, 1, 20, 20, ChCoordsys<>(ChVector<>(0.01, 0, 0.01), Q_from_AngX(CH_C_PI_2)),
                        ChColor(0.6f, 0.6f, 0.6f), true);
        tools::drawAllCOGs(vis.get(), 1.0);

        // Finalize the graphical scene.
        vis->EndScene();

        // Advance simulation by one step.
        system.DoStepDynamics(0.01);
        realtime_timer.Spin(0.01);
    }

    return 0;
}
