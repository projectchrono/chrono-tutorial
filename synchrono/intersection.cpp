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

#include <chrono>

#include "chrono_synchrono/cli/SynCLI.h"
#include "chrono_synchrono/communication/mpi/SynMPIManager.h"

#include "chrono_synchrono/framework/SynFramework.h"
#include "chrono_synchrono/utils/SynDataLoader.h"

#include "chrono_synchrono/agent/SynEnvironmentAgent.h"
#include "chrono_synchrono/agent/SynWheeledVehicleAgent.h"
#include "chrono_synchrono/brain/SynACCBrain.h"
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

std::shared_ptr<SynWheeledVehicle> InitializeVehicle(int rank);

const double lane1_x = 2.8;
const double lane2_x = 5.6;

// ------------------------------------------------------------------------------------

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
    SetSynChronoDataPath(std::string(CHRONO_DATA_DIR) + "synchrono/");

    // CLI tools for default synchrono demos
    // Setting things like step_size, simulation run-time, etc...
    SynCLI cli(argv[0]);
    cli.AddDefaultDemoOptions();
    if (!cli.Parse(argc, argv, rank == 0))
        mpi_manager.Exit();

    //// -------------------------------------------------------------------------
    //// EXERCISE 1
    //// Add a CityBus that will join our Sedan any time that we run with more
    //// than one rank
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

    // int traffic_light_rank = 2;
    // if (rank == traffic_light_rank) {
    //     // Do traffic light things

    //     // std::vector<ChVector<>> lane1_points = {{lane2_x, -15, 0.2}, {lane2_x, -40, 0.2}};
    // } else {
    //     // Do vehicle things
    // }

    // Here we make a vehicle and add it to the MPI manager on our rank
    // The InitializeVehicle function is just a nice wrapper to decide what vehicle we should have based on our rank
    auto agent = chrono_types::make_shared<SynWheeledVehicleAgent>(rank);
    agent->SetVehicle(InitializeVehicle(rank));
    mpi_manager.AddAgent(agent, rank);

    // -------
    // Terrain
    // -------
    auto terrain = chrono_types::make_shared<RigidTerrain>(agent->GetSystem());

    // Loading the mesh to be used for collisions
    auto patch = terrain->AddPatch(DefaultMaterialSurface(), CSYSNORM,
                                   GetSynDataFile("meshes/Highway_intersection.obj"), "", 0.01, false);

    // In this case the visualization mesh is the same, but it doesn't have to be (e.g. a detailed visual mesh of
    // buildings, but the collision mesh is just the driveable surface of the road)
    auto vis_mesh = chrono_types::make_shared<ChTriangleMeshConnected>();
    vis_mesh->LoadWavefrontMesh(GetSynDataFile("meshes/Highway_intersection.obj"), true, true);

    auto trimesh_shape = chrono_types::make_shared<ChTriangleMeshShape>();
    trimesh_shape->SetMesh(vis_mesh);
    trimesh_shape->SetStatic(true);

    patch->GetGroundBody()->AddAsset(trimesh_shape);

    terrain->Initialize();

    // Once we have an initialized ChTerrain, we wrap it in a SynRigidTerrain and attach it to our agent
    agent->SetTerrain(chrono_types::make_shared<SynRigidTerrain>(terrain));

    // ----------
    // Controller
    // ----------
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

    // -------------
    // Visualization
    // -------------
    auto vis_manager = chrono_types::make_shared<SynVisualizationManager>();

    // The visualization manager is a shared framework so that both Irrlicht and Sensor can place nicely under the hood
    agent->AttachVisualizationManager(vis_manager);

#ifdef CHRONO_IRRLICHT
    if (cli.HasValueInVector<int>("irr", rank)) {
        // Note that Irrlicht needs a driver to work off of
        auto irr_vis = chrono_types::make_shared<SynIrrVehicleVisualization>(acc_driver);
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

    // Send across initial messages telling the other ranks what agent type we are. It is blocking at both ends so you
    // can start timers after this point
    mpi_manager.Initialize();

    //// -------------------------------------------------------------------------
    //// EXERCISE 4
    //// Measure the fraction of real time (RTF) that the simulation runs in
    ////
    //// Some info:
    ////    - RTF = wall time / simulation time
    ////    -
    ////
    //// -------------------------------------------------------------------------

    // Simulation Loop - IsOk checks both that we haven't received some synchronization failure and that we aren't past
    // the end time
    while (mpi_manager.IsOk()) {
        // Advance does all the Chrono physics for our agent
        mpi_manager.Advance();

        // Synchronize has all ranks share their state data and any messages
        mpi_manager.Synchronize();

        // Update takes care of visualization of the zombies in our world
        mpi_manager.Update();

        // Exercise 2 details here...
    }

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
        // Exercise 1 details here...
        default:
            std::cerr << "No initial location specificied for this rank. Extra case needed?" << std::endl;
            filename = "vehicle/Sedan.json";
            init_rot = Q_from_AngZ(90 * CH_C_DEG_TO_RAD);
            init_loc = ChVector<>(lane1_x, -70, init_z);
            break;
    }

    // Filename specifies a json file with parameters for our vehicle (ego agent) and the zombie agents
    auto vehicle = chrono_types::make_shared<SynWheeledVehicle>(GetSynDataFile(filename), CONTACT_METHOD);
    vehicle->Initialize(ChCoordsys<>(init_loc, init_rot));

    return vehicle;
}