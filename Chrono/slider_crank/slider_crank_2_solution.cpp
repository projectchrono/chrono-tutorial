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
// Slider-crank Chrono tutorial (model 2)
//
// This model is a 3-body slider-crank consisting of crank, slider and connecting
// rod bodies. The crank is connected to ground with a revolute joint and the
// slider is connected to ground through a prismatic joint.  The connecting rod
// connects to the crank through a spherical joint and to the slider through a
// universal joint.
//
// The crank body is driven at constant angular speed, under the action of gravity,
// acting in the negative Z direction.
//
// An additional spherical body, constrained to move along the global X axis
// through a prismatic joint and connected to ground with a translational spring
// damper, interacts through contact with the slider body.
//
// The simulation is animated with Irrlicht.
//
// =============================================================================

#include <cmath>
#include <cstdio>

#include "chrono/assets/ChVisualShapePointPoint.h"
#include "chrono/physics/ChLinkMotorRotationSpeed.h"
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
    system.SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // Set an associated collision detection system
    system.SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // 2. Create the rigid bodies of the slider-crank mechanical system.
    //    For each body, specify:
    //    - a unique identifier
    //    - mass and moments of inertia
    //    - position and orientation of the (centroidal) body frame
    //    - visualization assets (defined with respect to the body frame)

    // Ground
    auto ground = chrono_types::make_shared<ChBody>();
    system.AddBody(ground);
    ground->SetName("ground");
    ground->SetFixed(true);

    auto cyl_g = chrono_types::make_shared<ChVisualShapeCylinder>(0.03, 0.4);
    ground->AddVisualShape(cyl_g, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));

    // Crank
    auto crank = chrono_types::make_shared<ChBody>();
    system.AddBody(crank);
    crank->SetName("crank");
    crank->SetMass(1.0);
    crank->SetInertiaXX(ChVector3d(0.005, 0.1, 0.1));
    crank->SetPos(ChVector3d(-1, 0, 0));
    crank->SetRot(ChQuaternion<>(1, 0, 0, 0));

    auto box_c = chrono_types::make_shared<ChVisualShapeBox>(1.9, 0.1, 0.1);
    box_c->SetColor(ChColor(0.6f, 0.2f, 0.2f));
    crank->AddVisualShape(box_c);

    auto cyl_c = chrono_types::make_shared<ChVisualShapeCylinder>(0.05, 0.2);
    cyl_c->SetColor(ChColor(0.6f, 0.2f, 0.2f));
    crank->AddVisualShape(cyl_c, ChFrame<>(ChVector3d(1, 0, 0), QuatFromAngleX(CH_PI_2)));

    auto sph_c = chrono_types::make_shared<ChVisualShapeSphere>(0.05);
    sph_c->SetColor(ChColor(0.6f, 0.2f, 0.2f));
    crank->AddVisualShape(sph_c, ChFrame<>(ChVector3d(-1, 0, 0), QUNIT));

    // Slider
    auto slider = chrono_types::make_shared<ChBody>();
    system.AddBody(slider);
    slider->SetName("slider");
    slider->SetMass(1.0);
    slider->SetInertiaXX(ChVector3d(0.05, 0.05, 0.05));
    slider->SetPos(ChVector3d(2, 0, 0));
    slider->SetRot(ChQuaternion<>(1, 0, 0, 0));

    auto box_s = chrono_types::make_shared<ChVisualShapeBox>(0.4, 0.2, 0.2);
    slider->AddVisualShape(box_s);

    auto cyl_s = chrono_types::make_shared<ChVisualShapeCylinder>(0.03, 0.4);
    cyl_s->SetColor(ChColor(0.2f, 0.2f, 0.6f));
    slider->AddVisualShape(cyl_s, ChFrame<>(VNULL, QuatFromAngleX(CH_PI_2)));

    // Connecting rod
    auto rod = chrono_types::make_shared<ChBody>();
    system.AddBody(rod);
    rod->SetName("rod");
    rod->SetMass(0.5);
    rod->SetInertiaXX(ChVector3d(0.005, 0.1, 0.1));
    rod->SetPos(ChVector3d(0, 0, 0));
    rod->SetRot(ChQuaternion<>(1, 0, 0, 0));

    auto box_r = chrono_types::make_shared<ChVisualShapeBox>(4, 0.1, 0.1);
    rod->AddVisualShape(box_r);

    auto cyl_r = chrono_types::make_shared<ChVisualShapeCylinder>(0.03, 0.4);
    cyl_r->SetColor(ChColor(0.2f, 0.6f, 0.2f));
    rod->AddVisualShape(cyl_r, ChFrame<>(ChVector3d(2, 0, 0), QuatFromAngleX(CH_PI_2)));

    //// -------------------------------------------------------------------------
    //// EXERCISE 2.1
    //// Enable contact on the slider body and specify contact geometry
    //// The contact shape attached to the slider body should be a box with the
    //// same dimensions as the visualization asset, centered at the body origin.
    //// Use a coefficient of friction of 0.4.
    //// -------------------------------------------------------------------------

    slider->EnableCollision(true);

    auto slider_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    slider_mat->SetFriction(0.4f);

    auto slider_ct_shape = chrono_types::make_shared<ChCollisionShapeBox>(slider_mat, 0.4, 0.2, 0.2);
    slider->AddCollisionShape(slider_ct_shape);

    //// -------------------------------------------------------------------------
    //// EXERCISE 2.2
    //// Create a new body, with a spherical shape (radius 0.2), used both as
    //// visualization asset and contact shape (mu = 0.4). This body should have:
    ////    mass: 1
    ////    moments of inertia: I_xx = I_yy = I_zz = 0.02
    ////    initial location: (5.5, 0, 0)
    //// -------------------------------------------------------------------------

    auto ball = chrono_types::make_shared<ChBody>();
    system.AddBody(ball);
    ball->SetName("ball");
    ball->SetMass(1);
    ball->SetInertiaXX(ChVector3d(0.02, 0.02, 0.02));
    ball->SetPos(ChVector3d(5.5, 0, 0));
    ball->SetRot(ChQuaternion<>(1, 0, 0, 0));

    ball->EnableCollision(true);

    // Contact material and collision shape
    auto ball_mat = chrono_types::make_shared<ChContactMaterialNSC>();
    auto ball_ct_shape = chrono_types::make_shared<ChCollisionShapeSphere>(ball_mat, 0.2);

    ball->AddCollisionShape(ball_ct_shape);

    auto sphere_b = chrono_types::make_shared<ChVisualShapeSphere>(0.2);
    sphere_b->SetColor(ChColor(0.6f, 0.6f, 0.6f));
    ball->AddVisualShape(sphere_b);

    // 3. Create joint constraints.
    //    All joint frames are specified in the global frame.

    // Define two quaternions representing:
    // - a rotation of -90 degrees around x (z2y)
    // - a rotation of +90 degrees around y (z2x)
    ChQuaternion<> z2y;
    ChQuaternion<> z2x;
    z2y.SetFromAngleAxis(-CH_PI / 2, ChVector3d(1, 0, 0));
    z2x.SetFromAngleAxis(CH_PI / 2, ChVector3d(0, 1, 0));

    // Create a ChFunction object that always returns the constant value PI/2.
    auto fun = chrono_types::make_shared<ChFunctionConst>();
    fun->SetConstant(CH_PI);

    // Motor between ground and crank.
    // Note that this also acts as a revolute joint (i.e. it enforces the same
    // kinematic constraints as a revolute joint).  As before, we apply the 'z2y'
    // rotation to align the rotation axis with the Y axis of the global frame.
    auto engine_ground_crank = chrono_types::make_shared<ChLinkMotorRotationSpeed>();
    engine_ground_crank->SetName("engine_ground_crank");
    engine_ground_crank->Initialize(ground, crank, ChFrame<>(ChVector3d(0, 0, 0), z2y));
    engine_ground_crank->SetSpeedFunction(fun);
    system.AddLink(engine_ground_crank);

    // Prismatic joint between ground and slider.
    // The translational axis of a prismatic joint is along the Z axis of the
    // specified joint coordinate system.  Here, we apply the 'z2x' rotation to
    // align it with the X axis of the global reference frame.
    auto prismatic_ground_slider = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic_ground_slider->SetName("prismatic_ground_slider");
    prismatic_ground_slider->Initialize(ground, slider, ChFrame<>(ChVector3d(2, 0, 0), z2x));
    system.AddLink(prismatic_ground_slider);

    // Spherical joint between crank and rod
    auto spherical_crank_rod = chrono_types::make_shared<ChLinkLockSpherical>();
    spherical_crank_rod->SetName("spherical_crank_rod");
    spherical_crank_rod->Initialize(crank, rod, ChFrame<>(ChVector3d(-2, 0, 0), QUNIT));
    system.AddLink(spherical_crank_rod);

    // Universal joint between rod and slider.
    // The "cross" of a universal joint is defined using the X and Y axes of the
    // specified joint coordinate frame. Here, we apply the 'z2x' rotation so that
    // the cross is aligned with the Z and Y axes of the global reference frame.
    auto universal_rod_slider = chrono_types::make_shared<ChLinkUniversal>();
    universal_rod_slider->SetName("universal_rod_slider");
    universal_rod_slider->Initialize(rod, slider, ChFrame<>(ChVector3d(2, 0, 0), z2x));
    system.AddLink(universal_rod_slider);

    //// -------------------------------------------------------------------------
    //// EXERCISE 2.3
    //// Add a prismatic joint between ground and ball to constrain the ball's
    //// motion to the global X axis.
    //// -------------------------------------------------------------------------

    auto prismatic_ground_ball = chrono_types::make_shared<ChLinkLockPrismatic>();
    prismatic_ground_ball->SetName("prismatic_ground_ball");
    prismatic_ground_ball->Initialize(ground, ball, ChFrame<>(ChVector3d(5.5, 0, 0), z2x));
    system.AddLink(prismatic_ground_ball);

    //// -------------------------------------------------------------------------
    //// EXERCISE 2.4
    //// Add a spring-damper (ChLinkTSDA) between ground and the ball.
    //// This element should connect the center of the ball with the global point
    //// (6.5, 0, 0).  Set a spring constant of 50 and a spring free length of 1.
    //// Set a damping coefficient of 5.
    //// -------------------------------------------------------------------------

    auto tsda_ground_ball = chrono_types::make_shared<ChLinkTSDA>();
    tsda_ground_ball->SetName("tsda_ground_ball");
    tsda_ground_ball->Initialize(ground, ball, false, ChVector3d(6.5, 0, 0), ChVector3d(5.5, 0, 0));
    tsda_ground_ball->SetSpringCoefficient(50.0);
    tsda_ground_ball->SetDampingCoefficient(5.0);
    tsda_ground_ball->SetRestLength(1.0);
    system.AddLink(tsda_ground_ball);

    tsda_ground_ball->AddVisualShape(chrono_types::make_shared<ChVisualShapeSpring>(0.05, 80, 15));

    // 4. Write the system hierarchy to the console (default log output destination)
    system.ShowHierarchy(std::cout);

    // 5. Prepare visualization with Irrlicht
    //    Note that Irrlicht uses left-handed frames with Y up.

    // Create the Irrlicht application and set-up the camera.
    auto vis = chrono_types::make_shared<ChVisualSystemIrrlicht>();
    vis->SetWindowSize(800, 600);
    vis->SetCameraVertical(CameraVerticalDir::Z);
    vis->SetWindowTitle("Slider-Crank Demo 2");
    vis->Initialize();
    vis->AddLogo();
    vis->AddSkyBox();
    vis->AddCamera(ChVector3d(2, -5, 0), ChVector3d(2, 0, 0));
    vis->AddTypicalLights();
    vis->AttachSystem(&system);

    // 6. Perform the simulation.

    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        // Initialize the graphical scene.
        vis->BeginScene();

        // Render all visualization objects.
        vis->Render();

        // Draw an XZ grid at the global origin to add in visualization.
        tools::drawGrid(vis.get(), 1, 1, 20, 20, ChCoordsys<>(ChVector3d(0.01, 0, 0.01), QuatFromAngleX(CH_PI_2)),
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
