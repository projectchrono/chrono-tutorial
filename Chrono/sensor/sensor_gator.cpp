// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Radu Serban
// =============================================================================
//
// Main driver function for the Gator full model.
//
// The vehicle reference frame has Z up, X towards the front of the vehicle, and
// Y pointing to the left.
//
// =============================================================================

#include "chrono/core/ChRealtimeStep.h"
#include "chrono/physics/ChBodyEasy.h"
#include "chrono/input_output/ChWriterCSV.h"
#include "chrono/input_output/ChUtilsInputOutput.h"

#include "chrono_vehicle/ChVehicleDataPath.h"
#include "chrono_vehicle/driver/ChInteractiveDriver.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "chrono_models/vehicle/gator/Gator.h"
#include "chrono_models/vehicle/gator/Gator_AutomaticTransmissionSimple.h"
#include "chrono_models/vehicle/gator/Gator_EngineSimple.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/sensors/ChGPSSensor.h"
#include "chrono_sensor/sensors/ChIMUSensor.h"
#include "chrono_sensor/sensors/ChLidarSensor.h"

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::gator;
using namespace chrono::sensor;

// =============================================================================

// Initial vehicle location and orientation
ChVector3d initLoc(0, 0, 0.5);
ChQuaternion<> initRot(1, 0, 0, 0);

// Visualization type for vehicle parts (PRIMITIVES, MESH, or NONE)
VisualizationType chassis_vis_type = VisualizationType::MESH;
VisualizationType suspension_vis_type = VisualizationType::PRIMITIVES;
VisualizationType steering_vis_type = VisualizationType::PRIMITIVES;
VisualizationType wheel_vis_type = VisualizationType::NONE;
VisualizationType tire_vis_type = VisualizationType::MESH;

// Collision type for chassis (PRIMITIVES, MESH, or NONE)
CollisionType collision_type = CollisionType::NONE;

// Type of tire model (RIGID, TMEASY)
TireModelType tire_model = TireModelType::TMEASY;

// Rigid terrain
RigidTerrain::PatchType terrain_model = RigidTerrain::PatchType::BOX;

// Contact method
ChContactMethod contact_method = ChContactMethod::NSC;

// Simulation step sizes
double step_size = 1e-3;
double tire_step_size = step_size;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// Output directories
const std::string out_dir = GetChronoOutputPath() + "Gator";
const std::string pov_dir = out_dir + "/POVRAY";
const std::string sens_dir = out_dir + "/SENSOR_OUTPUT";

// POV-Ray output
bool povray_output = false;

// ---------------
// Visualize sensor data
bool sensor_vis = true;

// Update rates of each sensor in Hz
float cam_update_rate = 30.f;
float lidar_update_rate = 10.f;
float exposure_time = 0.02f;
int super_samples = 2;

// Image width and height
unsigned int image_width = 1280;
unsigned int image_height = 720;

// Lidar horizontal and vertical samples
unsigned int horizontal_samples = 4500;
unsigned int vertical_samples = 32;

// Camera's horizontal field of view
float cam_fov = 1.408f;

// Lidar's horizontal and vertical fov
float lidar_hfov = (float)(2 * CH_PI);   // 360 degrees
float lidar_vmax = (float)(CH_PI / 12);  // 15 degrees up
float lidar_vmin = (float)(-CH_PI / 6);  // 30 degrees down

// =============================================================================

int main(int argc, char* argv[]) {
    std::cout << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << std::endl;

    // Path to Chrono data files (textures, etc.)
    SetChronoDataPath(CHRONO_DATA_DIR);

    // Path to the data files for this vehicle (JSON specification files)
    SetVehicleDataPath(GetChronoDataFile("") + "/vehicle/");

    // --------------
    // Create vehicle
    // --------------

    Gator gator;
    gator.SetContactMethod(contact_method);
    gator.SetChassisCollisionType(collision_type);
    gator.SetChassisFixed(false);
    gator.SetInitPosition(ChCoordsys<>(initLoc, initRot));
    gator.SetTireType(tire_model);
    gator.SetTireStepSize(tire_step_size);
    gator.SetAerodynamicDrag(0.5, 5.0, 1.2);
    gator.Initialize();
    gator.GetSystem()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    gator.SetChassisVisualizationType(chassis_vis_type);
    gator.SetSuspensionVisualizationType(suspension_vis_type);
    gator.SetSteeringVisualizationType(steering_vis_type);
    gator.SetWheelVisualizationType(wheel_vis_type);
    gator.SetTireVisualizationType(tire_vis_type);

    // ------------------
    // Create the terrain
    // ------------------

    RigidTerrain terrain(gator.GetSystem());

    ChContactMaterialData minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);

    std::shared_ptr<RigidTerrain::Patch> patch;
    switch (terrain_model) {
        case RigidTerrain::PatchType::BOX:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, 100.0, 100.0);
            patch->SetTexture(GetVehicleDataFile("terrain/textures/tile4.jpg"), 200, 200);
            break;
        case RigidTerrain::PatchType::HEIGHT_MAP:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, GetVehicleDataFile("terrain/height_maps/test64.bmp"), 128,
                                     128, 0, 4);
            patch->SetTexture(GetVehicleDataFile("terrain/textures/grass.jpg"), 16, 16);
            break;
        case RigidTerrain::PatchType::MESH:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, GetVehicleDataFile("terrain/meshes/test.obj"));
            patch->SetTexture(GetVehicleDataFile("terrain/textures/grass.jpg"), 100, 100);
            break;
    }
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

    terrain.Initialize();

    // ------------------------
    // Create the driver system
    // ------------------------

    // Create the interactive driver system
    ChInteractiveDriver driver(gator.GetVehicle());

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    driver.Initialize();

    // -------------------------------------
    // Create the vehicle Irrlicht interface
    // -------------------------------------

    auto vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
    vis->SetWindowTitle("Gator Demo");
    vis->SetChaseCamera(ChVector3d(0.0, 0.0, 2.0), 5.0, 0.05);
    vis->Initialize();
    vis->AddTypicalLights();
    vis->AddSkyBox();
    vis->AddLogo();
    vis->AttachVehicle(&gator.GetVehicle());
    vis->AttachDriver(&driver);

    // -----------------
    // Initialize output
    // -----------------

    if (povray_output) {
        if (!filesystem::create_directory(filesystem::path(out_dir))) {
            std::cout << "Error creating directory " << out_dir << std::endl;
            return 1;
        }
        if (!filesystem::create_directory(filesystem::path(pov_dir))) {
            std::cout << "Error creating directory " << pov_dir << std::endl;
            return 1;
        }
        terrain.ExportMeshPovray(out_dir);
    }

    // ------------------------------------------------------------------------------------------------------------
    // EXERCISE 1
    // Initialize Sensor System and Manager
    //          - create a sensor manager (ChSensorManager) associated with the chrono system (gator.GetSystem())
    //          - add a light to the scene in the manager
    //              - global position of lights = {100,100,100}
    //              - rgb intensity = {2,2,2}
    //              - range = 5000
    //          - Set the max key frame size from the simulation time step and the largest collection window
    // -------------------------------------------------------------------------------------------------------------

    //--------------------------------------------------------------------------------------------------------------
    // EXERCISE 2   (UNCOMMENT manager-Update() in the simualtion loop below!!!)
    //  Add a third person pov camera
    //          - attach to gator vehicle, gator.GetChassisBody()
    //          - offset = chrono::ChFrame<double>({-8, 0, 3}, QuatFromAngleAxis(.2, {0, 1, 0}))
    //          - updateRate, width, height, hFOV, super_samples are declared in ln103~ln119
    //          - Visualization with cam->PushFilter(.....)
    //          - Add sensor to manager
    //  Add a roof mounted camera
    //          - same steps above except
    //              -offset = chrono::ChFrame<double>({.1, 0, 1.45}, QuatFromAngleAxis(.2, {0, 1, 0}))
    //  BONUS
    //          - add noise before visualization
    //              - cam->PushFilter(chrono_types::make_shared<ChFilterCameraNoisePixDep>(0.f, .02f, .03f));
    //--------------------------------------------------------------------------------------------------------------

    //--------------------------------------------------------------------------------------------------------------
    // EXERCISE 3
    //  Add a lidar
    //          - attach to gator vehicle, gator.GetChassisBody()
    //          - offset = chrono::ChFrame<double>({-.282, 0, 1.82}, QuatFromAngleAxis(0, {1, 0, 0}))
    //          - lidar parameters declared in ln103~ln119
    //          - add sensor to manager
    //--------------------------------------------------------------------------------------------------------------

    // ---------------
    // Simulation loop
    // ---------------

    // output vehicle mass
    std::cout << "VEHICLE MASS: " << gator.GetVehicle().GetMass() << std::endl;

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;
    int render_frame = 0;

    ChRealtimeStepTimer realtime_timer;
    while (vis->Run()) {
        double time = gator.GetSystem()->GetChTime();

        // --------------------------------------------------------
        // BONUS
        //  Change the camera location so that is orbits vehicle
        // --------------------------------------------------------
        //  float orbit_radius = 15.f;
        //  float orbit_rate = 1;
        // 
        //  cam->SetOffsetPose(
        //      chrono::ChFrame<double>({-orbit_radius * cos(time * orbit_rate), -orbit_radius * sin(time * orbit_rate),
        //      3},
        //                              QuatFromAngleAxis(time * orbit_rate, {0, 0, 1})));

        // manager->Update();

        // ***************************** SENSOR CODE *******************************//

        // Render scene and output POV-Ray data
        if (step_number % render_steps == 0) {
            vis->BeginScene();
            vis->Render();
            vis->EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteVisualizationAssets(gator.GetSystem(), filename);
            }

            render_frame++;
        }

        // Get driver inputs
        DriverInputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        gator.Synchronize(time, driver_inputs, terrain);
        vis->Synchronize(time, driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        gator.Advance(step_size);
        vis->Advance(step_size);

        // Increment frame number
        step_number++;

        // Spin in place for real time to catch up
        realtime_timer.Spin(step_size);
    }

    return 0;
}
