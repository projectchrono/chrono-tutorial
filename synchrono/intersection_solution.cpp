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

#include <chrono>

#include "chrono_thirdparty/cxxopts/ChCLI.h"

#include "chrono_vehicle/driver/ChPathFollowerACCDriver.h"

#include "chrono_synchrono/communication/mpi/SynMPIManager.h"
#include "chrono_synchrono/utils/SynDataLoader.h"

#include "chrono_synchrono/agent/SynEnvironmentAgent.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/brain/SynACCBrain.h"
#include "chrono_synchrono/brain/SynEnvironmentBrain.h"
#include "chrono_synchrono/brain/driver/SynMultipathDriver.h"
#include "chrono_synchrono/terrain/SynRigidTerrain.h"
#include "chrono_synchrono/visualization/SynVisualizationManager.h"

#ifdef CHRONO_IRRLICHT
#include "chrono_synchrono/visualization/SynIrrVehicleVisualization.h"
#endif

#ifdef CHRONO_SENSOR
#include "chrono_synchrono/visualization/SynSensorVisualization.h"
#endif

using namespace chrono;
using namespace chrono::geometry;
using namespace chrono::synchrono;
using namespace chrono::vehicle;

std::shared_ptr<SynWheeledVehicle> InitializeVehicle(int rank);

const double lane1_x = 2.8;
const double lane2_x = 5.6;

// =============================================================================
const ChContactMethod contact_method = ChContactMethod::NSC;

// [s]
double end_time = 1000;
double step_size = 3e-3;

// Time interval between two render frames
double render_step_size = 1.0 / 50;  // FPS = 50

// How often SynChrono state messages are interchanged
float heartbeat = 1e-2;  // 100[Hz]

void AddCommandLineOptions(ChCLI& cli);

// =============================================================================

int main(int argc, char* argv[]) {
    // Initialize the MPIManager
    // After this point the code is being run once per rank
    SynMPIManager mpi_manager(argc, argv, MPI_CONFIG_DEFAULT);
    int rank = mpi_manager.GetRank();
    int num_ranks = mpi_manager.GetNumRanks();

    // Path to Chrono data files (textures, etc.)
    SetChronoDataPath(CHRONO_DATA_DIR);

    // Path to the data files for this demo (JSON specification files)
    vehicle::SetDataPath(std::string(CHRONO_DATA_DIR) + "vehicle/");
    synchrono::SetDataPath(std::string(CHRONO_DATA_DIR) + "synchrono/");

    // CLI tools for default synchrono demos
    // Setting things like step_size, simulation run-time, etc...
    ChCLI cli(argv[0]);

    AddCommandLineOptions(cli);

    if (!cli.Parse(argc, argv, rank == 0))
        mpi_manager.Exit();

    // Normal simulation options
    step_size = cli.GetAsType<double>("step_size");
    end_time = cli.GetAsType<double>("end_time");
    heartbeat = cli.GetAsType<double>("heartbeat");

    const bool use_sensor_vis = cli.HasValueInVector<int>("sens", rank);
    const bool use_irrlicht_vis = !use_sensor_vis && cli.HasValueInVector<int>("irr", rank);

    mpi_manager.SetHeartbeat(heartbeat);
    mpi_manager.SetEndTime(end_time);

    std::shared_ptr<SynWheeledVehicleAgent> agent;
    std::shared_ptr<ChMulPathFollowerACCDriver> multi_driver;

    int traffic_light_rank = 1;
    if (rank == traffic_light_rank) {
        // -------------
        // Traffic light
        // -------------

        auto agent = chrono_types::make_shared<SynEnvironmentAgent>(traffic_light_rank);
        mpi_manager.AddAgent(agent, traffic_light_rank);
        agent->SetBrain(chrono_types::make_shared<SynEnvironmentBrain>(traffic_light_rank));

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
    } else {
        // Here we make a vehicle and add it to the MPI manager on our rank
        // The InitializeVehicle function is just a nice wrapper to decide what vehicle we should have based on our rank
        agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank);
        agent->SetVehicle(InitializeVehicle(rank));
        mpi_manager.AddAgent(agent, rank);

        // -------
        // Terrain
        // -------
        MaterialInfo minfo;
        minfo.mu = 0.9f;
        minfo.cr = 0.01f;
        minfo.Y = 2e7f;
        auto patch_mat = minfo.CreateMaterial(contact_method);

        auto terrain = chrono_types::make_shared<RigidTerrain>(agent->GetSystem());

        // Loading the mesh to be used for collisions
        auto patch = terrain->AddPatch(patch_mat, CSYSNORM, synchrono::GetDataFile("meshes/Highway_intersection.obj"),
                                       "", 0.01, false);

        // In this case the visualization mesh is the same, but it doesn't have to be (e.g. a detailed visual mesh of
        // buildings, but the collision mesh is just the driveable surface of the road)
        auto vis_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
        vis_mesh->LoadWavefrontMesh(synchrono::GetDataFile("meshes/Highway_intersection.obj"), true, true);

        auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
        trimesh_shape->SetMesh(vis_mesh);
        trimesh_shape->SetStatic(true);

        patch->GetGroundBody()->AddAsset(trimesh_shape);

        terrain->Initialize();

        // Once we have an initialized ChTerrain, we wrap it in a SynRigidTerrain and attach it to our agent
        agent->SetTerrain(chrono_types::make_shared<SynRigidTerrain>(terrain));

        // -------------------
        // Vehicle Controllers
        // -------------------

        auto loc = agent->GetChVehicle().GetVehiclePos();

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

        if (rank != 0) {
            // Rank 2, 3, 4, ...
            auto acc_driver = chrono_types::make_shared<ChPathFollowerACCDriver>(
                agent->GetChVehicle(), path, "Highway", target_speed, target_following_time, target_min_distance,
                current_distance, is_path_closed);

            // Set some additional PID parameters and how far ahead along the bezier curve we should look
            acc_driver->GetSpeedController().SetGains(0.4, 0.0, 0.0);
            acc_driver->GetSteeringController().SetGains(0.4, 0.1, 0.2);
            acc_driver->GetSteeringController().SetLookAheadDistance(5);

            // Now that we have the 'acc_driver' (which is a ChDriver), we initialize a VehicleBrain using it
            auto brain = chrono_types::make_shared<SynVehicleBrain>(rank, acc_driver, agent->GetChVehicle());
            agent->SetBrain(brain);

            driver = acc_driver;
        } else {
            // Rank 0
            auto curve_pts2 = std::vector<ChVector<>>({ChVector<>(lane2_x, -70, 0.2), ChVector<>(5.8, 70, 0.2)});
            auto path2 = chrono_types::make_shared<ChBezierCurve>(curve_pts2);

            std::vector<std::pair<std::shared_ptr<ChBezierCurve>, bool>> path_pairs;
            path_pairs.push_back({path, false});
            path_pairs.push_back({path2, false});

            multi_driver = chrono_types::make_shared<ChMulPathFollowerACCDriver>(
                agent->GetChVehicle(), path_pairs, "Highway", target_speed, target_following_time, target_min_distance,
                current_distance);

            multi_driver->GetSpeedController().SetGains(0.4, 0.0, 0.0);
            multi_driver->GetSteeringController().SetGains(0.4, 0.1, 0.2);
            multi_driver->GetSteeringController().SetLookAheadDistance(5);

            auto brain = chrono_types::make_shared<SynACCBrain>(rank, multi_driver, agent->GetChVehicle());
            brain->setMultipath(true);
            agent->SetBrain(brain);

            driver = multi_driver;
        }

        // -------------
        // Visualization
        // -------------
        auto vis_manager = chrono_types::make_shared<SynVisualizationManager>();

        // The visualization manager is a shared framework so that both Irrlicht and Sensor can place nicely under the
        // hood
        agent->SetVisualizationManager(vis_manager);

#ifdef CHRONO_IRRLICHT
        if (cli.HasValueInVector<int>("irr", rank)) {
            // Note that Irrlicht needs a driver to work off of
            auto irr_vis = chrono_types::make_shared<SynIrrVehicleVisualization>(driver);
            irr_vis->InitializeAsDefaultChaseCamera(agent->GetVehicle());
            vis_manager->AddVisualization(irr_vis);
        }
#endif

#ifdef CHRONO_SENSOR
        if (cli.HasValueInVector<int>("sens", rank)) {
            std::string path = std::string("SENSOR_OUTPUT/magic") + std::to_string(rank) + std::string("/");

            std::shared_ptr<SynSensorVisualization> sen_vis = chrono_types::make_shared<SynSensorVisualization>();
            sen_vis->InitializeDefaultSensorManager(agent->GetSystem());
            sen_vis->InitializeAsDefaultChaseCamera(agent->GetChVehicle().GetChassisBody());

            // Save an image for each frame
            if (cli.GetAsType<bool>("sens_save"))
                sen_vis->AddFilterSave(path);

            // Display the camera's view to the screen (equivalent to Irrlicht above)
            if (cli.GetAsType<bool>("sens_vis"))
                sen_vis->AddFilterVisualize();

            vis_manager->AddVisualization(sen_vis);
        }
#endif
    }

    // Send across initial messages telling the other ranks what agent type we are. It is blocking at both ends so you
    // can start timers after this point
    mpi_manager.Barrier();
    mpi_manager.Initialize();

    ChTimer<> timer;
    timer.start();

    int step_number = 0;
    // Simulation Loop - IsOk checks both that we haven't received some synchronization failure and that we aren't past
    // the end time
    while (mpi_manager.IsOk()) {
        // Advance does all the Chrono physics for our agent
        mpi_manager.Advance(heartbeat * step_number++);

        // Synchronize has all ranks share their state data and any messages
        mpi_manager.Synchronize();

        // Update takes care of visualization of the zombies in our world
        mpi_manager.Update();

        if (rank == 0 && std::abs(agent->GetSystem()->GetChTime() - 2) < 1e-2)
            multi_driver->changePath(1);

        if (rank != traffic_light_rank) {
            auto pos = agent->GetChVehicle().GetVehicleCOMPos();
            // std::cout << "Position: " << pos.x() << ", " << pos.y() << ", " << pos.z() << std::endl;
        }
    }

    timer.stop();
    if (rank == 0)
        std::cout << "RTF: " << (timer.GetTimeSeconds() / end_time) << std::endl;

    // MPI_Finalize() is called in the destructor of mpi_manager
    return 0;
}

std::shared_ptr<SynWheeledVehicle> InitializeVehicle(int rank) {
    ChVector<> init_loc;
    ChQuaternion<> init_rot;
    std::string filename;

    double init_z = 0.5;
    switch (rank) {
        case 0:
            filename = "vehicle/Sedan.json";
            init_rot = Q_from_AngZ(90 * CH_C_DEG_TO_RAD);
            init_loc = ChVector<>(lane1_x, -70, init_z);
            break;
        case 1:
            filename = "vehicle/HMMWV.json";
            init_rot = Q_from_AngZ(90 * CH_C_DEG_TO_RAD);
            init_loc = ChVector<>(lane2_x, -70, init_z + 0.5);
            break;
        default:
            std::cerr << "No initial location specificied for this rank. Extra case needed?" << std::endl;
            filename = "vehicle/Sedan.json";
            init_rot = Q_from_AngZ(90 * CH_C_DEG_TO_RAD);
            init_loc = ChVector<>(lane1_x, -70, init_z);
            break;
    }
    ChCoordsys<> init_pos(init_loc, init_rot);

    // Filename specifies a json file with parameters for our vehicle (ego agent) and the zombie agents
    auto vehicle =
        chrono_types::make_shared<SynWheeledVehicle>(init_pos, synchrono::GetDataFile(filename), contact_method);

    return vehicle;
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