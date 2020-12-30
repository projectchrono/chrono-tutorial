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
#include "chrono/utils/ChUtilsInputOutput.h"
#include "chrono/physics/ChBodyEasy.h"

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/driver/ChIrrGuiDriver.h"
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"

#include "chrono_models/vehicle/gator/Gator.h"
#include "chrono_models/vehicle/gator/Gator_SimplePowertrain.h"

#include "chrono_thirdparty/filesystem/path.h"

#include "chrono_sensor/ChCameraSensor.h"
#include "chrono_sensor/ChLidarSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterPCfromDepth.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterSavePtCloud.h"
#include "chrono_sensor/filters/ChFilterVisualizePointCloud.h"
#include "chrono_sensor/utils/ChVisualMaterialUtils.h"
#include "chrono_sensor/ChGPSSensor.h"
#include "chrono_sensor/ChIMUSensor.h"
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
#include "chrono_sensor/filters/ChFilterCameraNoise.h"

using namespace chrono;
using namespace chrono::irrlicht;
using namespace chrono::vehicle;
using namespace chrono::vehicle::gator;
using namespace chrono::sensor;

// =============================================================================

// Initial vehicle location and orientation
ChVector<> initLoc(0, 0, 0.5);
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

// SENSOR PARAMETERS
// Save sensor data
bool sensor_save = false;

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
float lidar_hfov = (float)(2 * CH_C_PI);   // 360 degrees
float lidar_vmax = (float)(CH_C_PI / 12);  // 15 degrees up
float lidar_vmin = (float)(-CH_C_PI / 6);  // 30 degrees down

// ---------------

// =============================================================================

int main(int argc, char* argv[]) {
    GetLog() << "Copyright (c) 2017 projectchrono.org\nChrono version: " << CHRONO_VERSION << "\n\n";

    // Path to Chrono data files (textures, etc.)
    SetChronoDataPath(CHRONO_DATA_DIR);

    // Path to the data files for this vehicle (JSON specification files)
    vehicle::SetDataPath(GetChronoDataFile("") + "/vehicle/");

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

    gator.SetChassisVisualizationType(chassis_vis_type);
    gator.SetSuspensionVisualizationType(suspension_vis_type);
    gator.SetSteeringVisualizationType(steering_vis_type);
    gator.SetWheelVisualizationType(wheel_vis_type);
    gator.SetTireVisualizationType(tire_vis_type);

    // ------------------
    // Create the terrain
    // ------------------

    RigidTerrain terrain(gator.GetSystem());

    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);

    std::shared_ptr<RigidTerrain::Patch> patch;
    switch (terrain_model) {
        case RigidTerrain::PatchType::BOX:
            patch = terrain.AddPatch(patch_mat, ChVector<>(0, 0, 0), ChVector<>(0, 0, 1), 100.0, 100.0);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/tile4.jpg"), 200, 200);
            break;
        case RigidTerrain::PatchType::HEIGHT_MAP:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/height_maps/test64.bmp"),
                                     "test64", 128, 128, 0, 4);
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 16, 16);
            break;
        case RigidTerrain::PatchType::MESH:
            patch = terrain.AddPatch(patch_mat, CSYSNORM, vehicle::GetDataFile("terrain/meshes/test.obj"), "test_mesh");
            patch->SetTexture(vehicle::GetDataFile("terrain/textures/grass.jpg"), 100, 100);
            break;
    }
    patch->SetColor(ChColor(0.8f, 0.8f, 0.5f));

    terrain.Initialize();

    // -------------------------------------
    // Create the vehicle Irrlicht interface
    // -------------------------------------

    ChWheeledVehicleIrrApp app(&gator.GetVehicle(), L"Gator Demo");
    app.SetSkyBox();
    app.AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250, 130);
    app.SetChaseCamera(ChVector<>(0.0, 0.0, 2.0), 5.0, 0.05);
    app.SetTimestep(step_size);
    app.AssetBindAll();
    app.AssetUpdateAll();

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

    // ------------------------
    // Create the driver system
    // ------------------------

    // Create the interactive driver system
    ChIrrGuiDriver driver(app);

    // Set the time response for steering and throttle keyboard inputs.
    double steering_time = 1.0;  // time to go from 0 to +1 (or from 0 to -1)
    double throttle_time = 1.0;  // time to go from 0 to +1
    double braking_time = 0.3;   // time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time);
    driver.SetThrottleDelta(render_step_size / throttle_time);
    driver.SetBrakingDelta(render_step_size / braking_time);

    driver.Initialize();

    // sensor manager
    auto manager = chrono_types::make_shared<ChSensorManager>(gator.GetSystem());
    manager->scene->AddPointLight({100, 100, 100}, {2, 2, 2}, 5000);
    manager->SetKeyframeSizeFromTimeStep((float)step_size, exposure_time);

    // third person camera
    auto cam = chrono_types::make_shared<ChCameraSensor>(
        gator.GetChassisBody(),                                              // body camera is attached to
        cam_update_rate,                                                     // update rate in Hz
        chrono::ChFrame<double>({-8, 0, 3}, Q_from_AngAxis(.2, {0, 1, 0})),  // offset pose
        image_width,                                                         // image width
        image_height,                                                        // image height
        cam_fov,
        super_samples);  // fov, lag, exposure
    cam->SetName("3rd Person Camera Sensor");
    cam->SetCollectionWindow(exposure_time);
    // add camera noise model
    // cam->PushFilter(chrono_types::make_shared<ChFilterCameraNoisePixDep>(0.f, .02f, .03f));
    if (sensor_vis)
        cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height));
    if (sensor_save)
        cam->PushFilter(chrono_types::make_shared<ChFilterSave>(sens_dir + "/cam1/"));
    cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam);

    // roof mounted camera .1, 0, 1.45
    auto cam2 = chrono_types::make_shared<ChCameraSensor>(
        gator.GetChassisBody(),                                                 // body camera is attached to
        cam_update_rate,                                                        // update rate in Hz
        chrono::ChFrame<double>({.1, 0, 1.45}, Q_from_AngAxis(.2, {0, 1, 0})),  // offset pose
        image_width,                                                            // image width
        image_height,                                                           // image height
        cam_fov,
        super_samples);  // fov, lag, exposure
    cam2->SetName("Roof Mounted Camera Sensor");
    cam2->SetCollectionWindow(exposure_time);
    if (sensor_vis)
        cam2->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height));
    if (sensor_save)
        cam2->PushFilter(chrono_types::make_shared<ChFilterSave>(sens_dir + "/cam2/"));
    cam2->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());
    manager->AddSensor(cam2);

    auto lidar = chrono_types::make_shared<ChLidarSensor>(
        gator.GetChassisBody(),                                                   // body to which the IMU is attached
        lidar_update_rate,                                                        // update rate
        chrono::ChFrame<double>({-.282, 0, 1.82}, Q_from_AngAxis(0, {1, 0, 0})),  // offset pose from body
        horizontal_samples,                                                       // horizontal samples
        vertical_samples,                                                         // vertical samples/channels
        lidar_hfov,                                                               // horizontal field of view
        lidar_vmax,                                                               // vertical field of view
        lidar_vmin,                                                               // vertical field of view
        100.0,                                                                    //
        1,                                                                        //
        0,                                                                        //
        STRONGEST_RETURN,                                                         //
        RAYCAST,                                                                  //
        .1                                                                        //
    );
    lidar->SetName("Lidar Sensor");
    lidar->SetLag(1 / lidar_update_rate);
    lidar->SetCollectionWindow(0);
    lidar->PushFilter(chrono_types::make_shared<ChFilterPCfromDepth>());
    if (sensor_vis)
        lidar->PushFilter(chrono_types::make_shared<ChFilterVisualizePointCloud>(640, 480, .5, "Lidar Point Cloud"));
    if (sensor_save)
        lidar->PushFilter(chrono_types::make_shared<ChFilterSavePtCloud>(sens_dir + "/lidar/"));
    lidar->PushFilter(chrono_types::make_shared<ChFilterXYZIAccess>());
    manager->AddSensor(lidar);

    // ---------------
    // Simulation loop
    // ---------------

    // output vehicle mass
    std::cout << "VEHICLE MASS: " << gator.GetTotalMass() << std::endl;

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Initialize simulation frame counters
    int step_number = 0;
    int render_frame = 0;

    float orbit_radius = 10.f;
    float orbit_rate = 1;

    ChRealtimeStepTimer realtime_timer;
    while (app.GetDevice()->run()) {
        double time = gator.GetSystem()->GetChTime();

        // Change the camera location so that is orbits vehicle
        cam->SetOffsetPose(
            chrono::ChFrame<double>({-orbit_radius * cos(time * orbit_rate), -orbit_radius * sin(time * orbit_rate), 3},
                                    Q_from_AngAxis(time * orbit_rate, {0, 0, 1})));

        // Update the sensor manager -> this will perform all synchronize, render, augment functions in Chrono::Sensor
        manager->Update();

        // Render scene and output POV-Ray data
        if (step_number % render_steps == 0) {
            app.BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
            app.DrawAll();
            app.EndScene();

            if (povray_output) {
                char filename[100];
                sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), render_frame + 1);
                utils::WriteShapesPovray(gator.GetSystem(), filename);
            }

            render_frame++;
        }

        // Get driver inputs
        ChDriver::Inputs driver_inputs = driver.GetInputs();

        // Update modules (process inputs from other modules)
        driver.Synchronize(time);
        terrain.Synchronize(time);
        gator.Synchronize(time, driver_inputs, terrain);
        app.Synchronize(driver.GetInputModeAsString(), driver_inputs);

        // Advance simulation for one timestep for all modules
        driver.Advance(step_size);
        terrain.Advance(step_size);
        gator.Advance(step_size);
        app.Advance(step_size);

        // Increment frame number
        step_number++;

        // Spin in place for real time to catch up
        realtime_timer.Spin(step_size);
    }

    return 0;
}
