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
// Chrono::Multicore tutorial.
//
// The model simulated here consists of a spherical projectile dropped in a
// bed of granular material, using either penalty or complementarity method for
// frictional contact.
//
// The global reference frame has Z up.
// All units SI.
// =============================================================================

#include <cstdio>
#include <vector>
#include <cmath>

#include "chrono/ChConfig.h"
#include "chrono/utils/ChUtilsCreators.h"
#include "chrono/utils/ChUtilsGenerators.h"

#include "chrono_multicore/physics/ChSystemMulticore.h"

#ifdef CHRONO_OPENGL
#include "chrono_opengl/ChVisualSystemOpenGL.h"
#endif

using namespace chrono;

// -----------------------------------------------------------------------------
// Problem definitions
// -----------------------------------------------------------------------------

// Desired number of OpenMP threads (will be clamped to maximum available)
int threads = 4;

// Parameters for the granular material
int tag_particles = 0;  // all particles wilkl have tag at least this value
double r_g = 2e-3;
double rho_g = 2500;
double vol_g = (4.0 / 3) * CH_PI * r_g * r_g * r_g;
double mass_g = rho_g * vol_g;
ChVector3d inertia_g = 0.4 * mass_g * r_g * r_g * ChVector3d(1, 1, 1);

float Y_g = 1e7f;
float mu_g = 0.3f;
float cr_g = 0.1f;

// Parameters for the falling ball
double R_b = 1.5e-2;
double rho_b = 700;
double vol_b = (4.0 / 3) * CH_PI * R_b * R_b * R_b;
double mass_b = rho_b * vol_b;
ChVector3d inertia_b = 0.4 * mass_b * R_b * R_b * ChVector3d(1, 1, 1);

float Y_b = 1e8f;
float mu_b = 0.3f;
float cr_b = 0.1f;

// Parameters for the containing bin
double hDimX = 4e-2;         // length in x direction
double hDimY = 4e-2;         // depth in y direction
double hDimZ = 4e-2;         // height in z direction
double hThickness = 0.5e-2;  // wall thickness

float Y_c = 2e6f;
float mu_c = 0.3f;
float cr_c = 0.1f;

// Drop height and vertical velocity
double initial_height = 0.1;
double initial_velocity = 0;

// -----------------------------------------------------------------------------
// Create the container (five boxes)
// -----------------------------------------------------------------------------
void CreateContainer(ChSystemMulticore* system) {
    // Create a material for the container
    auto material_c = chrono_types::make_shared<ChContactMaterialNSC>();
    material_c->SetFriction(mu_c);

    // Create the container. This utility function creates the container body (fixed to "ground")
    // and sets both the contact and visualization shapes.
    utils::CreateBoxContainer(system, material_c, ChVector3d(hDimX, hDimY, hDimZ), hThickness);
}

// -----------------------------------------------------------------------------
// Create the falling ball at the specified height, with specified vertical
// initial velocity.
// -----------------------------------------------------------------------------
std::shared_ptr<ChBody> CreateFallingBall(ChSystemMulticore* system) {
    // Create a contact material for the falling ball
    auto material_b = chrono_types::make_shared<ChContactMaterialNSC>();
    material_b->SetFriction(mu_b);

    // Create the falling ball body
    auto ball = chrono_types::make_shared<ChBody>();

    ball->SetMass(mass_b);
    ball->SetInertiaXX(inertia_b);
    ball->SetPos(ChVector3d(0, 0, initial_height));
    ball->SetRot(ChQuaternion<>(1, 0, 0, 0));
    ball->SetLinVel(ChVector3d(0, 0, initial_velocity));
    ball->EnableCollision(true);
    ball->SetFixed(false);

    // Specify spherical contact and visualization shapes
    utils::AddSphereGeometry(ball.get(), material_b, R_b);

    system->AddBody(ball);

    return ball;
}

// -----------------------------------------------------------------------------
// Create the granular material, consisting of identical spheres with given
// radius and material properties; the spheres are generated in a prismatic
// region inside the container, using Poisson Disk sampling (thus ensuring that
// no two spheres are closer than twice the radius)
// -----------------------------------------------------------------------------
void CreateObjects(ChSystemMulticore* system) {
    // Create a contact material for granular bodies
    auto material_g = chrono_types::make_shared<ChContactMaterialNSC>();
    material_g->SetFriction(mu_g);

    //// ********************************************************************************
    //// EXERCISE:
    //// Create a granular material generator with a mixture entirely made out of spheres
    //// of equal radius, all sharing the same contact material
    //// ********************************************************************************
}

// -----------------------------------------------------------------------------
int main(int argc, char* argv[]) {
    // Set the path to the Chrono data folder.
    SetChronoDataPath(CHRONO_DATA_DIR);

    // Create the (multicore) system and set method-specific solver settings.
    ChSystemMulticore* system = new ChSystemMulticoreNSC;

    system->SetCollisionSystemType(ChCollisionSystem::Type::MULTICORE);

    system->GetSettings()->solver.solver_type = SolverType::BB;
    system->GetSettings()->solver.solver_mode = SolverMode::SLIDING;
    system->GetSettings()->solver.max_iteration_normal = 0;
    system->GetSettings()->solver.max_iteration_sliding = 100;
    system->GetSettings()->solver.max_iteration_spinning = 0;
    system->GetSettings()->solver.alpha = 0;
    system->GetSettings()->solver.contact_recovery_speed = 0.1;

    double time_step = 1e-3;

    // Set number of threads.
    int max_threads = omp_get_num_procs();
    if (threads > max_threads)
        threads = max_threads;
    omp_set_num_threads(threads);
    std::cout << "Using " << threads << " threads" << std::endl;

    // Set gravitational acceleration
    system->SetGravitationalAcceleration(ChVector3d(0, 0, -9.81));

    // Set method-independent solver settings
    system->GetSettings()->solver.use_full_inertia_tensor = false;
    system->GetSettings()->solver.tolerance = 1.0;

    system->GetSettings()->collision.narrowphase_algorithm = ChNarrowphase::Algorithm::HYBRID;
    system->GetSettings()->collision.collision_envelope = 0.05 * r_g;

    //// ******************************************************
    //// EXERCISE:
    //// Provide a more efficient split of bins for broad-phase
    //// ******************************************************
    system->GetSettings()->collision.bins_per_axis = vec3(1, 1, 1);

    // Create the objects
    CreateContainer(system);
    auto ball = CreateFallingBall(system);
    CreateObjects(system);

    // Simulation loop

#ifdef CHRONO_OPENGL
    // If Chrono::openGL is available, create the visualization window
    opengl::ChVisualSystemOpenGL vis;
    vis.AttachSystem(system);
    vis.SetWindowTitle("Crater Test");
    vis.SetWindowSize(1280, 720);
    vis.SetRenderMode(opengl::WIREFRAME);
    vis.Initialize();
    vis.AddCamera(ChVector3d(0, -7 * hDimY, hDimZ), ChVector3d(0, 0, hDimZ));
    vis.SetCameraVertical(CameraVerticalDir::Z);

    // Run simulation loop (ESC to terminate)
    while (true) {
        if (vis.Run()) {
            system->DoStepDynamics(time_step);
            vis.Render();
        } else {
            break;
        }
    }

    const ChVector3d& pos = ball->GetPos();
    std::cout << system->GetChTime() << "  " << pos.z() << std::endl;
#else
    // If Chrono::openGL is not available, run simulation to specified end time
    double time_end = 5;
    int out_steps = static_cast<int>(std::ceil(1.0 / time_step) / 100);
    int sim_frame = 0;
    while (system->GetChTime() < time_end) {
        if (sim_frame % out_steps == 0) {
            const ChVector3d& pos = ball->GetPos();
            std::cout << system->GetChTime() << "  " << pos.z() << std::endl;
        }
        system->DoStepDynamics(time_step);
        sim_frame++;
    }
#endif

    delete system;
    return 0;
}
