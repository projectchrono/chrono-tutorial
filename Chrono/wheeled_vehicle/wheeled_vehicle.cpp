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
// Authors: Radu Serban
// =============================================================================
//
// Main driver function for a wheeled vehicle specified through JSON files.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/ChConfig.h"
#include "chrono/core/ChRealtimeStep.h"
#include "chrono/core/ChRandom.h"
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/core/ChRandom.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#include "chrono_vehicle/powertrain/EngineSimple.h"
#include "chrono_vehicle/powertrain/AutomaticTransmissionSimpleMap.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChInteractiveDriver.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

using namespace chrono;

// =============================================================================

std::string vehicle_file("vehicle/WheeledVehicle.json");
std::string tire_file("vehicle/RigidTire.json");
std::string engine_file("vehicle/EngineSimple.json");
std::string transmission_file("vehicle/AutomaticTransmissionSimpleMap.json");

std::string rigidterrain_file("terrain/RigidPlane.json");

// Initial vehicle position and orientation
ChVector3d initLoc(0, 0, 1.0);
ChQuaternion<> initRot(1, 0, 0, 0);

// Simulation step size
double step_size = 2e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Point on chassis tracked by the camera
ChVector3d trackPoint(0.0, 0.0, 1.75);

// Contact method
auto NSC_SMC = ChContactMethod::NSC;

// =============================================================================

void AddMovingObstacles(ChSystem* system);
void AddFixedObstacles(ChSystem* system);

// =============================================================================

int main(int argc, char* argv[]) {
    // ----------------------
    // Set path to data files
    // ----------------------

    // Path to Chrono data files (textures, etc.)
    SetChronoDataPath(CHRONO_DATA_DIR);

    // Path to the data files for this vehicle (JSON specification files)
    vehicle::SetDataPath(std::string(SOURCE_DIR) + "/data/");

    // --------------------------
    // Create the various modules
    // --------------------------
    
    // Create and initialize the vehicle system
    vehicle::WheeledVehicle vehicle(vehicle::GetDataFile(vehicle_file), NSC_SMC);

    vehicle.Initialize(ChCoordsys<>(initLoc, initRot));

    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::NONE);

    // Create the terrain
    vehicle::RigidTerrain terrain(vehicle.GetSystem(), vehicle::GetDataFile(rigidterrain_file));
    terrain.Initialize();
    AddFixedObstacles(vehicle.GetSystem());
    AddMovingObstacles(vehicle.GetSystem());

    // Create and initialize the powertrain system
    auto engine = vehicle::ReadEngineJSON(vehicle::GetDataFile(engine_file));
    auto transmission = vehicle::ReadTransmissionJSON(vehicle::GetDataFile(transmission_file));
    auto powertrain = chrono_types::make_shared<vehicle::ChPowertrainAssembly>(engine, transmission);
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle.GetAxles()) {
        auto tireL = vehicle::ReadTireJSON(vehicle::GetDataFile(tire_file));
        auto tireR = vehicle::ReadTireJSON(vehicle::GetDataFile(tire_file));
        vehicle.InitializeTire(tireL, axle->m_wheels[0]);
        vehicle.InitializeTire(tireR, axle->m_wheels[1]);
    }

    // Set collision detection system
    vehicle.GetSystem()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    // Create the driver system (interactive)
    vehicle::ChInteractiveDriver driver(vehicle);
    driver.SetSteeringDelta(0.02);
    driver.SetThrottleDelta(0.02);
    driver.SetBrakingDelta(0.06);
    driver.Initialize();

    // Create the Irrlicht vehicle application
    auto vis = chrono_types::make_shared<vehicle::ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Vehicle Demo");
    vis->SetChaseCamera(trackPoint, 5.0, 0.5);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&vehicle);
    vis->AttachDriver(&driver);

    // ---------------
    // Simulation loop
    // ---------------

    // Initialize simulation frame counter and simulation time
    double time = 0;

    vehicle.EnableRealtime(true);

    while (vis->Run()) {
        // Render scene
        vis->BeginScene();
        vis->Render();
        vis->EndScene();

        // Get driver inputs
        vehicle::DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        time = vehicle.GetSystem()->GetChTime();
        driver.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
        terrain.Synchronize(time);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        vehicle.Advance(step_size);
        terrain.Advance(step_size);
        vis->Advance(step_size);
    }

    return 0;
}

void AddMovingObstacles(ChSystem* system) {
    // Create contact material, of appropriate type. Use default properties
    std::shared_ptr<ChContactMaterial> material;
    switch (NSC_SMC) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChContactMaterialNSC>();
            // Change NSC material properties as desired
            material = matNSC;
            break;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChContactMaterialSMC>();
            // Change SMC material properties as desired
            material = matSMC;
            break;
        }
    }

    double sizeX = 300;
    double sizeY = 300;
    double height = 0;
    int numObstacles = 10;

    for (int i = 0; i < numObstacles; i++) {
        double o_sizeX = 1.0 + 6.0 * ChRandom::Get();
        double o_sizeY = 0.3 + 0.4 * ChRandom::Get();
        double o_sizeZ = 0.05 + 0.2 * ChRandom::Get();
        auto obstacle = chrono_types::make_shared<ChBodyEasyBox>(o_sizeX, o_sizeY, o_sizeZ, 2000.0, true, true, material);

        double o_posX = (ChRandom::Get() - 0.5) * 0.4 * sizeX;
        double o_posY = (ChRandom::Get() - 0.5) * 0.4 * sizeY;
        double o_posZ = height + 4;
        ChQuaternion<> rot(ChRandom::Get(), ChRandom::Get(), ChRandom::Get(), ChRandom::Get());
        rot.Normalize();
        obstacle->SetPos(ChVector3d(o_posX, o_posY, o_posZ));
        obstacle->SetRot(rot);

        system->AddBody(obstacle);
    }
}

void AddFixedObstacles(ChSystem* system) {
    // Create contact material, of appropriate type. Use default properties
    std::shared_ptr<ChContactMaterial> material;
    switch (NSC_SMC) {
        case ChContactMethod::NSC: {
            auto matNSC = chrono_types::make_shared<ChContactMaterialNSC>();
            // Change NSC material properties as desired
            material = matNSC;
            break;
        }
        case ChContactMethod::SMC: {
            auto matSMC = chrono_types::make_shared<ChContactMaterialSMC>();
            // Change SMC material properties as desired
            material = matSMC;
            break;
        }
    }

    double radius = 3;
    double length = 20;
    auto obstacle = chrono_types::make_shared<ChBodyEasyCylinder>(ChAxis::Y, radius, length, 2000, true, true, material);

    obstacle->SetPos(ChVector3d(-20, 0, -2.7));
    obstacle->SetFixed(true);

    system->AddBody(obstacle);

    for (int i = 0; i < 8; ++i) {
        auto stoneslab = chrono_types::make_shared<ChBodyEasyBox>(1.0, 5.0, 0.5, 2000, true, true, material);
        stoneslab->SetPos(ChVector3d(-1.2 * i + 22, -1.5, -0.25));
        stoneslab->SetRot(QuatFromAngleAxis(15 * CH_DEG_TO_RAD, VECT_Y));
        stoneslab->SetFixed(true);
        system->AddBody(stoneslab);
    }
}
