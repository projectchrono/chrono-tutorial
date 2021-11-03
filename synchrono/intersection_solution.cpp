// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2020 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Jay Taves
// =============================================================================
//
// Demo code of vehicles on a highway, used for hands-on exercises at MaGIC 2020
// These are the solutions and contain the completed tutorial
//
// =============================================================================

#include "chrono_models/vehicle/citybus/CityBus.h"
#include "chrono_models/vehicle/sedan/Sedan.h"

#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/agent/SynEnvironmentAgent.h"
#include "chrono_synchrono/controller/driver/SynMultiPathDriver.h"
#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"
#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/utils/SynLog.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "chrono_vehicle/wheeled_vehicle/vehicle/WheeledVehicle.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerACCDriver.h"
#include "chrono_vehicle/utils/ChUtilsJSON.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleIrrApp.h"
using namespace chrono::irrlicht;
#endif

#ifdef CHRONO_SENSOR
#include "chrono_sensor/ChSensorManager.h"
#include "chrono_sensor/sensors/ChCameraSensor.h"
#include "chrono_sensor/filters/ChFilterAccess.h"
#include "chrono_sensor/filters/ChFilterSave.h"
#include "chrono_sensor/filters/ChFilterVisualize.h"
using namespace chrono::sensor;
#endif

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::synchrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::sedan;

const double lane1_x = 2.8;
const double lane2_x = 5.6;

// Image dimensions for Chrono::Sensor
const unsigned int image_width = 1280;
const unsigned int image_height = 720;

// =============================================================================
const ChContactMethod contact_method = ChContactMethod::NSC;

// [s]
double end_time = 1000;
double step_size = 3e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// How often SynChrono state messages are interchanged
float heartbeat = 1e-2f;  // 100[Hz]

struct VehInfo {
    std::string vehicle_filename;
    std::string powertrain_filename;
    std::string tire_filename;
    std::string zombie_filename;

    ChCoordsys<> init_pose;
} VehInfo;

void AddCommandLineOptions(ChCLI& cli);
void LogCopyright(bool show);
struct VehInfo InitializeVehicle(int node_id);
void ProcessTrafficLightMessage(std::shared_ptr<SynMessage> msg);

// =============================================================================

int main(int argc, char* argv[]) {
    // Initialize the MPIManager
    // After this point the code is being run once per node
    auto communicator = chrono_types::make_shared<SynMPICommunicator>(argc, argv);
    int node_id = communicator->GetRank();
    int num_nodes = communicator->GetNumRanks();
    SynChronoManager syn_manager(node_id, num_nodes, communicator);

    // Copyright
    LogCopyright(node_id == 0);

    // Path to Chrono data files (textures, etc.)
    SetChronoDataPath(CHRONO_DATA_DIR);

    // Path to the data files for this demo (JSON specification files)
    vehicle::SetDataPath(std::string(CHRONO_DATA_DIR) + "vehicle/");
    synchrono::SetDataPath(std::string(CHRONO_DATA_DIR) + "synchrono/");

    // CLI tools for default synchrono demos
    // Setting things like step_size, simulation run-time, etc...
    ChCLI cli(argv[0]);

    AddCommandLineOptions(cli);
    if (!cli.Parse(argc, argv, node_id == 0))
        return 0;

    // Normal simulation options
    step_size = cli.GetAsType<double>("step_size");
    end_time = cli.GetAsType<double>("end_time");
    heartbeat = cli.GetAsType<float>("heartbeat");

    const bool use_sensor = cli.HasValueInVector<int>("sens", node_id);
    const bool sensor_vis = cli.GetAsType<bool>("sens_vis");
    const bool sensor_save = cli.GetAsType<bool>("sens_save");
    const bool use_irrlicht_vis = cli.HasValueInVector<int>("irr", node_id);

    syn_manager.SetHeartbeat(heartbeat);

    // This convenience function wraps a large case statement that switches based on node_id
    struct VehInfo veh_info = InitializeVehicle(node_id);

    WheeledVehicle vehicle(veh_info.vehicle_filename, contact_method);
    vehicle.Initialize(veh_info.init_pose);
    vehicle.GetChassis()->SetFixed(false);
    vehicle.SetChassisVisualizationType(VisualizationType::MESH);
    vehicle.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    vehicle.SetWheelVisualizationType(VisualizationType::MESH);

    // Create and initialize the powertrain system
    auto powertrain = ReadPowertrainJSON(veh_info.powertrain_filename);
    vehicle.InitializePowertrain(powertrain);

    // Create and initialize the tires
    for (auto& axle : vehicle.GetAxles()) {
        for (auto& wheel : axle->GetWheels()) {
            auto tire = ReadTireJSON(veh_info.tire_filename);
            vehicle.InitializeTire(tire, wheel, VisualizationType::MESH);
        }
    }

    ChSystem* system = vehicle.GetSystem();

    // Add vehicle as an agent
    syn_manager.AddAgent(chrono_types::make_shared<SynWheeledVehicleAgent>(&vehicle, veh_info.zombie_filename));

    // -------------------
    // Traffic Light
    // -------------------
    int traffic_light_node = 1;
    if (node_id == traffic_light_node) {
        // -------------
        // Traffic light
        // -------------

        auto agent = chrono_types::make_shared<SynEnvironmentAgent>(system);
        syn_manager.AddAgent(agent);

        double approach_start_y = -40;  // [m]
        double approach_end_y = -15;    // [m]
        double lane_width = 2.5;        // [m]

        double red_time = 10;
        double yellow_time = 1;
        double green_time = 5;

        std::vector<double> schedule1 = {red_time, yellow_time, green_time};
        std::vector<ChVector<>> lane1_points = {{lane2_x, approach_start_y, 0.2}, {lane2_x, approach_end_y, 0.2}};
        ApproachLane lane_1(lane_width, lane1_points);

        agent->AddLane(0, 0, lane_1, LaneColor::RED, schedule1);
    }

    syn_manager.Initialize(system);

    // -------
    // Terrain
    // -------
    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);

    RigidTerrain terrain(system);

    // Loading the mesh to be used for collisions
    auto patch =
        terrain.AddPatch(patch_mat, CSYSNORM, synchrono::GetDataFile("meshes/Highway_intersection.obj"), 0.01, false);

    // In this case the visualization mesh is the same, but it doesn't have to be (e.g. a detailed visual mesh of
    // buildings, but the collision mesh is just the driveable surface of the road)
    auto vis_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    vis_mesh->LoadWavefrontMesh(synchrono::GetDataFile("meshes/Highway_intersection.obj"), true, true);

    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(vis_mesh);
    trimesh_shape->SetStatic(true);

    patch->GetGroundBody()->AddAsset(trimesh_shape);

    terrain.Initialize();

    // -------------------
    // Vehicle Controllers
    // -------------------

    auto loc = vehicle.GetVehiclePos();

    // These two points just define a straight line in the direction the vehicle is oriented
    auto curve_pts = std::vector<ChVector<>>({loc, loc + ChVector<>(0, 140, 0)});
    auto path = chrono_types::make_shared<ChBezierCurve>(curve_pts);

    // These are all parameters for a ChPathFollowerACCDriver
    double target_speed = 10;            // [m/s]
    double target_following_time = 1.2;  // [s]
    double target_min_distance = 10;     // [m]
    double current_distance = 100;       // [m]
    bool is_path_closed = false;

    std::shared_ptr<ChDriver> driver;

    if (node_id != 0) {
        // Nodes 2, 3, 4, ...
        auto acc_driver = chrono_types::make_shared<ChPathFollowerACCDriver>(vehicle, path, "Highway", target_speed,
                                                                             target_following_time, target_min_distance,
                                                                             current_distance, is_path_closed);

        // Set some additional PID parameters and how far ahead along the bezier curve we should look
        acc_driver->GetSpeedController().SetGains(0.4, 0.0, 0.0);
        acc_driver->GetSteeringController().SetGains(0.4, 0.1, 0.2);
        acc_driver->GetSteeringController().SetLookAheadDistance(5);

        driver = acc_driver;
    } else {
        // Node 0
        auto curve_pts2 = std::vector<ChVector<>>({ChVector<>(lane2_x, -70, 0.2), ChVector<>(5.8, 70, 0.2)});
        auto path2 = chrono_types::make_shared<ChBezierCurve>(curve_pts2);

        std::vector<std::pair<std::shared_ptr<ChBezierCurve>, bool>> path_pairs = {{path, false}, {path2, false}};

        auto multi_driver = chrono_types::make_shared<ChMultiPathFollowerACCDriver>(
            vehicle, path_pairs, "Highway", target_speed, target_following_time, target_min_distance, current_distance);

        multi_driver->GetSpeedController().SetGains(0.4, 0.0, 0.0);
        multi_driver->GetSteeringController().SetGains(0.4, 0.1, 0.2);
        multi_driver->GetSteeringController().SetLookAheadDistance(5);

        driver = multi_driver;
    }

#ifdef CHRONO_IRRLICHT
    // Create the vehicle Irrlicht interface
    std::shared_ptr<ChWheeledVehicleIrrApp> app;
    if (use_irrlicht_vis) {
        ChVector<> track_point(0.0, 0.0, 1.75);

        app = chrono_types::make_shared<ChWheeledVehicleIrrApp>(&vehicle, L"SynChrono Vehicle Demo");
        app->SetSkyBox();
        app->AddTypicalLights(irr::core::vector3df(30.f, -30.f, 100.f), irr::core::vector3df(30.f, 50.f, 100.f), 250,
                              130);
        app->SetChaseCamera(track_point, 6.0, 0.5);
        app->SetTimestep(step_size);
        app->AssetBindAll();
        app->AssetUpdateAll();
    }
#endif

#ifdef CHRONO_SENSOR
    ChSensorManager sensor_manager(system);
    if (use_sensor) {
        sensor_manager.scene->AddPointLight({100, 100, 100}, {2, 2, 2}, 5000);

        // Based on Irrlicht chase cam - slightly above and behind the vehicle
        chrono::ChFrame<double> chase_pose({-6, 0, 1.5}, Q_from_AngAxis(0, {1, 0, 0}));
        auto chase_cam =
            chrono_types::make_shared<ChCameraSensor>(vehicle.GetChassisBody(),  // body camera is attached to
                                                      30.0f,                     // update rate in Hz
                                                      chase_pose,                // offset pose
                                                      image_width,               // image width
                                                      image_height,              // image height
                                                      1.408f,                    // horizontal fov
                                                      2);                        // supersample
        chase_cam->SetName("Camera Sensor");

        // Renders the image
        if (sensor_vis)
            chase_cam->PushFilter(chrono_types::make_shared<ChFilterVisualize>(image_width, image_height));

        // Save the current image to a png file at the specified path
        if (sensor_save) {
            const std::string out_dir = GetChronoOutputPath() + "/SENSOR_OUTPUT/magic/";
            chase_cam->PushFilter(chrono_types::make_shared<ChFilterSave>(out_dir));
        }

        // Provides the host access to this RGBA8 buffer
        chase_cam->PushFilter(chrono_types::make_shared<ChFilterRGBA8Access>());

        // add sensor to the manager
        sensor_manager.AddSensor(chase_cam);
    }
#endif

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);
    int step_number = 0;

    ChTimer<> timer;
    timer.start();

    while (true) {
        double time = system->GetChTime();

        if (time >= end_time)
            break;

#ifdef CHRONO_IRRLICHT
        if (use_irrlicht_vis) {
            if (!app->GetDevice()->run())
                break;

            // Render scene and output POV-Ray data
            if (step_number % render_steps == 0 && app) {
                app->BeginScene(true, true, irr::video::SColor(255, 140, 161, 192));
                app->DrawAll();
                app->EndScene();
            }
        }
#endif  // CHRONO_IRRLICHT

        // Get driver inputs
        ChDriver::Inputs driver_inputs = driver->GetInputs();

        // SynChrono update step - everything else is normal Chrono::Vehicle steps
        syn_manager.Synchronize(time);  // Synchronize between nodes

        driver->Synchronize(time);
        terrain.Synchronize(time);
        vehicle.Synchronize(time, driver_inputs, terrain);
#ifdef CHRONO_IRRLICHT
        if (use_irrlicht_vis)
            app->Synchronize("", driver_inputs);
#endif

        // Advance simulation for one timestep for all modules
        driver->Advance(step_size);
        terrain.Advance(step_size);
        vehicle.Advance(step_size);
#ifdef CHRONO_IRRLICHT
        if (use_irrlicht_vis)
            app->Advance(step_size);
#endif

#ifdef CHRONO_SENSOR
        sensor_manager.Update();
#endif  // SENSOR

        // Increment frame number
        step_number++;

        if (node_id == 0 && std::abs(time - 2) < 1e-2) {
            auto obj = std::dynamic_pointer_cast<ChMultiPathFollowerACCDriver>(driver);
            if (obj) {
                obj->changePath(1);
            } else {
                std::cout << "Dynamic cast failed" << std::endl;
            }
        }

        if (node_id != traffic_light_node) {
            auto pos = vehicle.GetVehicleCOMPos();
            // std::cout << "Position: " << pos.x() << ", " << pos.y() << ", " << pos.z() << std::endl;
        }
    }

    timer.stop();
    if (node_id == 0)
        std::cout << "RTF: " << (timer.GetTimeSeconds() / end_time) << std::endl;

    // MPI_Finalize() is called in the destructor of mpi_manager
    return 0;
}

struct VehInfo InitializeVehicle(int node_id) {
    ChVector<> init_loc;
    ChQuaternion<> init_rot = Q_from_AngZ(90 * CH_C_DEG_TO_RAD);
    double init_z = 0.5;
    struct VehInfo info;
    switch (node_id) {
        case 0:
            info.vehicle_filename = vehicle::GetDataFile("sedan/vehicle/Sedan_Vehicle.json");
            info.powertrain_filename = vehicle::GetDataFile("sedan/powertrain/Sedan_SimpleMapPowertrain.json");
            info.tire_filename = vehicle::GetDataFile("sedan/tire/Sedan_TMeasyTire.json");
            info.zombie_filename = synchrono::GetDataFile("vehicle/Sedan.json");

            init_loc = ChVector<>(lane1_x, -70, init_z);

            info.init_pose = ChCoordsys<>(init_loc, init_rot);
            break;
        case 1:
            info.vehicle_filename = vehicle::GetDataFile("citybus/vehicle/CityBus_Vehicle.json");
            info.powertrain_filename = vehicle::GetDataFile("citybus/powertrain/CityBus_SimpleMapPowertrain.json");
            info.tire_filename = vehicle::GetDataFile("citybus/tire/CityBus_TMeasyTire.json");
            info.zombie_filename = synchrono::GetDataFile("vehicle/CityBus.json");

            init_loc = ChVector<>(lane2_x, -70, init_z + 0.5);

            info.init_pose = ChCoordsys<>(init_loc, init_rot);
            break;
        default:
            std::cerr << "No initial location specificied for node " << node_id << ". Extra case needed?" << std::endl;
            break;
    }

    return info;
}

void LogCopyright(bool show) {
    if (!show)
        return;

    SynLog() << "Copyright (c) 2020 projectchrono.org\n";
    SynLog() << "Chrono version: " << CHRONO_VERSION << "\n\n";
}

void AddCommandLineOptions(ChCLI& cli) {
    // Standard demo options
    cli.AddOption<double>("Simulation", "step_size", "Step size", std::to_string(step_size));
    cli.AddOption<double>("Simulation", "end_time", "End time", std::to_string(end_time));
    cli.AddOption<double>("Simulation", "heartbeat", "Heartbeat", std::to_string(heartbeat));

    // Irrlicht options
    cli.AddOption<std::vector<int>>("Irrlicht", "irr", "Nodes for irrlicht usage", "-1");

    // Sensor options
    cli.AddOption<std::vector<int>>("Sensor", "sens", "Nodes for sensor usage", "-1");
    cli.AddOption<bool>("Sensor", "sens_save", "Toggle sensor saving ON", "false");
    cli.AddOption<bool>("Sensor", "sens_vis", "Toggle sensor visualization ON", "false");
}