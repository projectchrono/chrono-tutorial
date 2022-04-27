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
//
// =============================================================================
#include "chrono_models/vehicle/sedan/Sedan.h"

#include "chrono_synchrono/SynConfig.h"
#include "chrono_synchrono/SynChronoManager.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/communication/mpi/SynMPICommunicator.h"
#include "chrono_synchrono/utils/SynDataLoader.h"
#include "chrono_synchrono/utils/SynLog.h"

#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/driver/ChPathFollowerACCDriver.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_vehicle/wheeled_vehicle/utils/ChWheeledVehicleVisualSystemIrrlicht.h"
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

void AddCommandLineOptions(ChCLI& cli);
void LogCopyright(bool show);

// =============================================================================

int main(int argc, char* argv[]) {
    // Initialize the MPIManager
    // After this point the code is being run once per rank
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

    //// -------------------------------------------------------------------------
    //// EXERCISE 1
    //// Add a CityBus that will join our Sedan any time that we run with more
    //// than one node_id
    //// This vehicle should:
    ////    Be a CityBus - "vehicle/CityBus.json"
    ////    Start one lane over - x, y = (lane2_x, -70)
    ////    Follow a straight path like the Sedan
    ////    Go a bit slower (say 5 m/s)
    //// -------------------------------------------------------------------------

    //// -------------------------------------------------------------------------
    //// EXERCISE 3
    //// Add an EnvironmentAgent at the intersection to act as a smart Traffic Light
    //// The traffic light will need:
    ////    A single approach
    ////    A single lane - say, the one that the Sedan merges into
    ////    To stay red for a while, then turn green
    ////
    //// Commented out below is some structure that can help you get started
    //// -------------------------------------------------------------------------

    // int traffic_light_node = 2;
    // if (node_id == traffic_light_node) {
    //     // Do traffic light things

    //     // std::vector<ChVector<>> lane1_points = {{lane2_x, -15, 0.2}, {lane2_x, -40, 0.2}};
    // } else {
    //     // Do vehicle things
    // }

    ChVector<> init_loc;
    ChQuaternion<> init_rot;
    double init_z = 0.5;
    switch (node_id) {
        case 0:
            init_rot = Q_from_AngZ(90 * CH_C_DEG_TO_RAD);
            init_loc = ChVector<>(lane1_x, -70, init_z);
            break;
        case 1:
            init_rot = Q_from_AngZ(90 * CH_C_DEG_TO_RAD);
            init_loc = ChVector<>(lane2_x, -70, init_z + 0.5);
            break;
        default:
            std::cerr << "No initial location specificied for this rank. Extra case needed?" << std::endl;
            break;
    }

    // Here we make a vehicle and add it to the MPI manager on our rank
    // The InitializeVehicle function is just a nice wrapper to decide what vehicle we should have based on our rank
    Sedan my_sedan;
    my_sedan.SetContactMethod(contact_method);
    my_sedan.SetChassisFixed(false);
    my_sedan.SetInitPosition(ChCoordsys<>(init_loc, init_rot));
    my_sedan.SetTireType(TireModelType::TMEASY);
    my_sedan.SetTireStepSize(step_size);
    my_sedan.Initialize();

    my_sedan.SetChassisVisualizationType(VisualizationType::MESH);
    my_sedan.SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    my_sedan.SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    my_sedan.SetWheelVisualizationType(VisualizationType::MESH);
    my_sedan.SetTireVisualizationType(VisualizationType::MESH);

    // Add vehicle as an agent and initialize SynChronoManager
    std::string zombie_filename = synchrono::GetDataFile("vehicle/Sedan.json");
    syn_manager.AddAgent(chrono_types::make_shared<SynWheeledVehicleAgent>(&my_sedan.GetVehicle(), zombie_filename));
    syn_manager.Initialize(my_sedan.GetSystem());

    // -------
    // Terrain
    // -------
    MaterialInfo minfo;
    minfo.mu = 0.9f;
    minfo.cr = 0.01f;
    minfo.Y = 2e7f;
    auto patch_mat = minfo.CreateMaterial(contact_method);

    RigidTerrain terrain(my_sedan.GetSystem());

    // Loading the mesh to be used for collisions
    auto patch = terrain.AddPatch(patch_mat, CSYSNORM, synchrono::GetDataFile("meshes/Highway_intersection.obj"), true,
                                  0.01, false);

    // In this case the visualization mesh is the same, but it doesn't have to be (e.g. a detailed visual mesh of
    // buildings, but the collision mesh is just the driveable surface of the road)
    auto vis_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    vis_mesh->LoadWavefrontMesh(synchrono::GetDataFile("meshes/Highway_intersection.obj"), true, true);

    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(vis_mesh);
    trimesh_shape->SetMutable(false);

    patch->GetGroundBody()->AddVisualShape(trimesh_shape);

    terrain.Initialize();

    // ----------
    // Controller
    // ----------

    // These two points just define a straight line in the direction the vehicle is oriented
    auto curve_pts = std::vector<ChVector<>>({init_loc, init_loc + ChVector<>(0, 140, 0)});
    auto path = chrono_types::make_shared<ChBezierCurve>(curve_pts);

    // These are all parameters for a ChPathFollowerACCDriver
    double target_speed = node_id == 0 ? 10 : 5;  // [m/s]
    double target_following_time = 1.2;           // [s]
    double target_min_distance = 10;              // [m]
    double current_distance = 100;                // [m]
    bool is_path_closed = false;

    //// -------------------------------------------------------------------------
    //// EXERCISE 2
    //// Make our Sedan change lanes after a certain point in time
    ////
    //// Some info:
    ////    - The other lane is along x = lane2_x (see CityBus initialization)
    ////    - ChMulPathFollowerACCDriver (say that 5 times fast...) takes a vector
    ////        of pairs of <shared_ptr<ChBezierCurve>, is_path_closed> defining
    ////        several lanes
    ////    - The vehicle needs to know when to change lanes driver->changePath
    ////    - Having it change after 2 seconds is a good amount of time
    ////
    //// -------------------------------------------------------------------------

    ChPathFollowerACCDriver acc_driver(my_sedan.GetVehicle(), path, "Highway", target_speed, target_following_time,
                                       target_min_distance, current_distance, is_path_closed);

    // Set some additional PID parameters and how far ahead along the bezier curve we should look
    acc_driver.GetSpeedController().SetGains(0.4, 0.0, 0.0);
    acc_driver.GetSteeringController().SetGains(0.4, 0.1, 0.2);
    acc_driver.GetSteeringController().SetLookAheadDistance(5);

    // -------------
    // Visualization
    // -------------

#ifdef CHRONO_IRRLICHT
    // Create the vehicle Irrlicht interface
    std::shared_ptr<ChWheeledVehicleVisualSystemIrrlicht> vis;
    if (use_irrlicht_vis) {
        ChVector<> track_point(0.0, 0.0, 1.75);

        vis = chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
        vis->SetWindowTitle("SynChrono Vehicle Demo");
        vis->SetChaseCamera(track_point, 6.0, 0.5);
        vis->Initialize();
        vis->AddTypicalLights();
        vis->AddSkyBox();
        vis->AddLogo();
        my_sedan.GetVehicle().SetVisualSystem(vis);
    }
#endif

#ifdef CHRONO_SENSOR
    ChSensorManager sensor_manager(my_sedan.GetSystem());
    if (use_sensor) {
        sensor_manager.scene->AddPointLight({100, 100, 100}, {2, 2, 2}, 5000);

        // Based on Irrlicht chase cam - slightly above and behind the vehicle
        chrono::ChFrame<double> chase_pose({-6, 0, 1.5}, Q_from_AngAxis(0, {1, 0, 0}));
        auto chase_cam =
            chrono_types::make_shared<ChCameraSensor>(my_sedan.GetChassisBody(),  // body camera is attached to
                                                      30.0f,                      // update rate in Hz
                                                      chase_pose,                 // offset pose
                                                      image_width,                // image width
                                                      image_height,               // image height
                                                      1.408f,                     // horizontal fov
                                                      2);                         // supersample
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

    //// -------------------------------------------------------------------------
    //// EXERCISE 4
    //// Measure the fraction of real time (RTF) that the simulation runs in
    ////
    //// Some info:
    ////    - RTF = wall time / simulation time
    ////    -
    ////
    //// -------------------------------------------------------------------------

    // Number of simulation steps between miscellaneous events
    int render_steps = (int)std::ceil(render_step_size / step_size);

    // Simulation Loop
    int step_number = 0;
    while (true) {
        double time = my_sedan.GetSystem()->GetChTime();

        if (time >= end_time)
            break;

#ifdef CHRONO_IRRLICHT
        if (use_irrlicht_vis) {
            if (!vis->Run())
                break;

            // Render scene and output POV-Ray data
            if (step_number % render_steps == 0 && vis) {
                vis->BeginScene();
                vis->DrawAll();
                vis->EndScene();
            }
        }
#endif  // CHRONO_IRRLICHT

        // Get driver inputs
        ChDriver::Inputs driver_inputs = acc_driver.GetInputs();

        // SynChrono update step - everything else is normal Chrono::Vehicle steps
        syn_manager.Synchronize(time);  // Synchronize between nodes

        acc_driver.Synchronize(time);
        terrain.Synchronize(time);
        my_sedan.Synchronize(time, driver_inputs, terrain);
#ifdef CHRONO_IRRLICHT
        if (use_irrlicht_vis)
            vis->Synchronize("", driver_inputs);
#endif

        // Advance simulation for one timestep for all modules
        acc_driver.Advance(step_size);
        terrain.Advance(step_size);
        my_sedan.Advance(step_size);
#ifdef CHRONO_IRRLICHT
        if (use_irrlicht_vis)
            vis->Advance(step_size);
#endif

#ifdef CHRONO_SENSOR
        sensor_manager.Update();
#endif  // SENSOR

        // Increment frame number
        step_number++;

        // Exercise 2 details here...
    }

    // MPI_Finalize() is called in the destructor of syn_manager
    return 0;
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
    cli.AddOption<std::vector<int>>("Irrlicht", "irr", "Ranks for irrlicht usage", "-1");

    // Sensor options
    cli.AddOption<std::vector<int>>("Sensor", "sens", "Ranks for sensor usage", "-1");
    cli.AddOption<bool>("Sensor", "sens_save", "Toggle sensor saving ON", "false");
    cli.AddOption<bool>("Sensor", "sens_vis", "Toggle sensor visualization ON", "false");
}