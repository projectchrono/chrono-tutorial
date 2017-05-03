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
#include "chrono/physics/ChSystemSMC.h"
#include "chrono/physics/ChSystem.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono/core/ChFileutils.h"
#include "chrono/utils/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleModelData.h"

#include "chrono_vehicle/powertrain/SimplePowertrain.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/wheeled_vehicle/tire/RigidTire.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

using namespace chrono;

// =============================================================================

std::string vehicle_file("vehicle/WheeledVehicle.json");
std::string rigidtire_file("vehicle/RigidTire.json");
std::string simplepowertrain_file("vehicle/SimplePowertrain.json");

std::string rigidterrain_file("terrain/RigidPlane.json");

// Initial vehicle position and orientation
ChVector<> initLoc(0, 0, 1.0);
ChQuaternion<> initRot(1, 0, 0, 0);

// Simulation step size
double step_size = 2e-4;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Point on chassis tracked by the camera
ChVector<> trackPoint(0.0, 0.0, 1.75);

// Contact method
auto NSC_SMC = ChMaterialSurface::SMC;

// Output directories
const std::string out_dir = "../WHEELED_VEHICLE";
const std::string pov_dir = out_dir + "/POVRAY";

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

    vehicle.SetSuspensionVisualizationType(vehicle::VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(vehicle::VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(vehicle::VisualizationType::NONE);

    // Create the terrain
    vehicle::RigidTerrain terrain(vehicle.GetSystem(), vehicle::GetDataFile(rigidterrain_file));
    AddFixedObstacles(vehicle.GetSystem());
    AddMovingObstacles(vehicle.GetSystem());

    // Create and initialize the powertrain system
    vehicle::SimplePowertrain powertrain(vehicle::GetDataFile(simplepowertrain_file));
    powertrain.Initialize(vehicle.GetChassis()->GetBody(), vehicle.GetDriveshaft());

    // Create and initialize the tires
    int num_axles = vehicle.GetNumberAxles();
    int num_wheels = 2 * num_axles;
    std::vector<std::shared_ptr<vehicle::RigidTire>> tires(num_wheels);

    for (int i = 0; i < num_wheels; i++) {
        std::cout << "tire " << i << std::endl;
        tires[i] = std::make_shared<vehicle::RigidTire>(vehicle::GetDataFile(rigidtire_file));
        tires[i]->Initialize(vehicle.GetWheelBody(i), vehicle::VehicleSide(i % 2));
        tires[i]->SetVisualizationType(vehicle::VisualizationType::MESH);
    }

    // Create the Irrlicht vehicle application
    vehicle::ChVehicleIrrApp app(&vehicle, &powertrain, L"Vehicle Demo");

    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(trackPoint, 6.0, 0.5);

    app.SetTimestep(step_size);

    app.AssetBindAll();
    app.AssetUpdateAll();

    // Create the driver system (interactive)
    vehicle::ChIrrGuiDriver driver(app);

    // Set the time response for steering and throttle keyboard inputs.
    // NOTE: this is not exact, since we do not render quite at the specified FPS.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    // -----------------
    // Initialize output
    // -----------------

    if (ChFileutils::MakeDirectory(out_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << out_dir << std::endl;
        return 1;
    }
    if (ChFileutils::MakeDirectory(pov_dir.c_str()) < 0) {
        std::cout << "Error creating directory " << pov_dir << std::endl;
        return 1;
    }

    // ---------------
    // Simulation loop
    // ---------------

    // Inter-module communication data
    vehicle::TireForces tire_forces(num_wheels);
    vehicle::WheelStates wheel_states(num_wheels);
    double driveshaft_speed;
    double powertrain_torque;
    double throttle_input;
    double steering_input;
    double braking_input;

    // Number of simulation steps between two 3D view render frames
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counter and simulation time
    int step_number = 0;
    int frame_number = 0;
    double time = 0;

    ChRealtimeStepTimer realtime_timer;

    while (app.GetDevice()->run()) {
        // Render scene
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();

            char filename[100];
            sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), frame_number + 1);
            utils::WriteShapesPovray(vehicle.GetSystem(), filename);

            frame_number++;
        }

        // Collect output data from modules (for inter-module communication)
        throttle_input = driver.GetThrottle();
        steering_input = driver.GetSteering();
        braking_input = driver.GetBraking();
        powertrain_torque = powertrain.GetOutputTorque();
        driveshaft_speed = vehicle.GetDriveshaftSpeed();
        for (int i = 0; i < num_wheels; i++) {
            tire_forces[i] = tires[i]->GetTireForce();
            wheel_states[i] = vehicle.GetWheelState(i);
        }

        // Update modules (process inputs from other modules)
        time = vehicle.GetSystem()->GetChTime();
        driver.Synchronize(time);
        powertrain.Synchronize(time, throttle_input, driveshaft_speed);
        vehicle.Synchronize(time, steering_input, braking_input, powertrain_torque, tire_forces);
        terrain.Synchronize(time);
        for (int i = 0; i < num_wheels; i++)
            tires[i]->Synchronize(time, wheel_states[i], terrain);
        app.Synchronize(driver.GetInputModeAsString(), steering_input, throttle_input, braking_input);

        // Advance simulation for one timestep for all modules
        double step = realtime_timer.SuggestSimulationStep(step_size);
        driver.Advance(step);
        powertrain.Advance(step);
        vehicle.Advance(step);
        terrain.Advance(step);
        for (int i = 0; i < num_wheels; i++)
            tires[i]->Advance(step);
        app.Advance(step);

        // Increment frame number
        step_number++;
    }

    return 0;
}

void AddMovingObstacles(ChSystem* system) {
    double sizeX = 300;
    double sizeY = 300;
    double height = 0;
    int numObstacles = 10;

    for (int i = 0; i < numObstacles; i++) {
        double o_sizeX = 1.0 + 3.0 * ChRandom();
        double o_sizeY = 0.3 + 0.2 * ChRandom();
        double o_sizeZ = 0.05 + 0.1 * ChRandom();
        auto obstacle = std::make_shared<ChBodyEasyBox>(o_sizeX, o_sizeY, o_sizeZ, 2000.0, true, true, NSC_SMC);

        double o_posX = (ChRandom() - 0.5) * 0.4 * sizeX;
        double o_posY = (ChRandom() - 0.5) * 0.4 * sizeY;
        double o_posZ = height + 4;
        ChQuaternion<> rot(ChRandom(), ChRandom(), ChRandom(), ChRandom());
        rot.Normalize();
        obstacle->SetPos(ChVector<>(o_posX, o_posY, o_posZ));
        obstacle->SetRot(rot);

        system->AddBody(obstacle);
    }
}

void AddFixedObstacles(ChSystem* system) {
    double radius = 3;
    double length = 10;
    auto obstacle = std::make_shared<ChBodyEasyCylinder>(radius, length, 2000, true, true, NSC_SMC);

    obstacle->SetPos(ChVector<>(-20, 0, -2.7));
    obstacle->SetBodyFixed(true);

    system->AddBody(obstacle);

    for (int i = 0; i < 8; ++i) {
        auto stoneslab = std::make_shared<ChBodyEasyBox>(0.5, 2.5, 0.25, 2000, true, true, NSC_SMC);
        stoneslab->SetPos(ChVector<>(-1.2 * i + 22, -1.5, -0.05));
        stoneslab->SetRot(Q_from_AngAxis(15 * CH_C_DEG_TO_RAD, VECT_Y));
        stoneslab->SetBodyFixed(true);
        system->AddBody(stoneslab);
    }
}
