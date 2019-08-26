# -*- coding: utf-8 -*-
"""
Created on Wed Jun  5 10:14:45 2019

@author: SB
"""

import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os
import math as m


def AddMovingObstacles(system) :
    sizeX = 300
    sizeY = 300
    height = 0
    numObstacles = 10

    for i in range(numObstacles) :
        o_sizeX = 1.0 + 3.0 * chrono.ChRandom()
        o_sizeY = 0.3 + 0.2 * chrono.ChRandom()
        o_sizeZ = 0.05 + 0.1 * chrono.ChRandom()
        obstacle = chrono.ChBodyEasyBox(o_sizeX, o_sizeY, o_sizeZ, 2000.0, True, True, chrono.ChMaterialSurface.SMC)

        o_posX = (chrono.ChRandom() - 0.5) * 0.4 * sizeX
        o_posY = (chrono.ChRandom() - 0.5) * 0.4 * sizeY
        o_posZ = height + 4
        rot = chrono.ChQuaternionD(chrono.ChRandom(), chrono.ChRandom(), chrono.ChRandom(), chrono.ChRandom())
        rot.Normalize()
        obstacle.SetPos(chrono.ChVectorD(o_posX, o_posY, o_posZ))
        obstacle.SetRot(rot)

        system.AddBody(obstacle)
    
    

def AddFixedObstacles(system) :
    radius = 3;
    length = 10;
    obstacle = chrono.ChBodyEasyCylinder(radius, length, 2000, True, True, chrono.ChMaterialSurface.SMC)

    obstacle.SetPos(chrono.ChVectorD(-20, 0, -2.7))
    obstacle.SetBodyFixed(True)

    system.AddBody(obstacle)

    for i in range(8) :
        stoneslab = chrono.ChBodyEasyBox(0.5, 2.5, 0.25, 2000, True, True, chrono.ChMaterialSurface.SMC)
        stoneslab.SetPos(chrono.ChVectorD(-1.2 * i + 22, -1.5, -0.05))
        
        stoneslab.SetRot(chrono.Q_from_AngAxis(15 * chrono.CH_C_DEG_TO_RAD, chrono.VECT_Y))
        stoneslab.SetBodyFixed(True);
        system.AddBody(stoneslab);
        


def main():
    # ----------------------
    # Set path to data files
    # ----------------------

    # Path to Chrono data files (textures, etc.)
    chrono.SetChronoDataPath(CHRONO_DATA_DIR);

    # Path to the data files for this vehicle (JSON specification files)
    veh.SetDataPath("./data/")

    # --------------------------
    # Create the various modules
    # --------------------------
    
    # Create and initialize the vehicle system
    vehicle = veh.WheeledVehicle(vehicle_file, chrono.ChMaterialSurface.SMC)

    vehicle.Initialize(chrono.ChCoordsysD(initLoc, initRot))

    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)

    # Create the terrain
    terrain = veh.RigidTerrain(vehicle.GetSystem(), veh.GetDataFile(rigidterrain_file))
    AddFixedObstacles(vehicle.GetSystem())
    AddMovingObstacles(vehicle.GetSystem())

    # Create and initialize the powertrain system
    powertrain = veh.SimplePowertrain(veh.GetDataFile(simplepowertrain_file))
    powertrain.Initialize(vehicle.GetChassis().GetBody(), vehicle.GetDriveshaft());

    # Create and initialize the tires
    num_axles = vehicle.GetNumberAxles()
    num_wheels = 2 * num_axles
    
    tires = [ veh.RigidTire(rigidtire_file) for i in range(num_wheels)]

    for i, t in enumerate(tires):
       print( "tire ", i ,'\n')
       
        #t = std::make_shared<vehicle::RigidTire>(vehicle::GetDataFile(rigidtire_file));
       s = [veh.LEFT, veh.RIGHT]
       t.Initialize(vehicle.GetWheelBody(veh.WheelID(i)), s[i % 2])
       t.SetVisualizationType(veh.VisualizationType_MESH)
    
       

    # Create the Irrlicht vehicle application
    app = veh.ChVehicleIrrApp(vehicle, powertrain)

    app.SetSkyBox()
    app.AddTypicalLights(chronoirr.vector3df(30, -30, 100), chronoirr.vector3df(30, 50, 100), 250, 130)
    app.SetChaseCamera(trackPoint, 6.0, 0.5)

    app.SetTimestep(step_size)

    app.AssetBindAll()
    app.AssetUpdateAll()

    # Create the driver system (interactive)
    driver = veh.ChIrrGuiDriver(app)

    # Set the time response for steering and throttle keyboard inputs.
    # NOTE: this is not exact, since we do not render quite at the specified FPS.
    steering_time = 1.0;  # time to go from 0 to +1 (or from 0 to -1)
    throttle_time = 1.0;  # time to go from 0 to +1
    braking_time = 0.3;   # time to go from 0 to +1
    driver.SetSteeringDelta(render_step_size / steering_time)
    driver.SetThrottleDelta(render_step_size / throttle_time)
    driver.SetBrakingDelta(render_step_size / braking_time)

    # -----------------
    # Initialize output
    # -----------------

    try:
           os.mkdir(out_dir)
    except:
           print("Error creating directory " )
    
              
    if (povray_output):
        try: 
            os.mkdir(pov_dir)
        except:
            print("Error creating POV directory ")
        terrain.ExportMeshPovray(out_dir)

    # ---------------
    # Simulation loop
    # ---------------

    # Inter-module communication data
    tire_forces = veh.TerrainForces(num_wheels)
    wheel_states = veh.WheelStates (num_wheels)
    
    """driveshaft_speed
    powertrain_torque
    throttle_input
    steering_input
    braking_input"""

    # Number of simulation steps between two 3D view render frames
    render_steps = m.ceil(render_step_size / step_size)

    #Initialize simulation frame counter and simulation time
    step_number = 0
    frame_number = 0
    time = 0

    realtime_timer = chrono.ChRealtimeStepTimer()

    while (app.GetDevice().run())  :
        # Render scene
        if (step_number % render_steps == 0) :
            app.BeginScene(True, True, chronoirr.SColor(255, 140, 161, 192))
            app.DrawAll();
            app.EndScene();

            #char filename[100];
            #sprintf(filename, "%s/data_%03d.dat", pov_dir.c_str(), frame_number + 1);
            #utils::WriteShapesPovray(vehicle.GetSystem(), filename);

            frame_number += 1
        

        # Collect output data from modules (for inter-module communication)
        throttle_input = driver.GetThrottle()
        steering_input = driver.GetSteering()
        braking_input = driver.GetBraking()
        powertrain_torque = powertrain.GetOutputTorque()
        driveshaft_speed = vehicle.GetDriveshaftSpeed()
        for i in range (num_wheels) :
            tire_forces[i] = tires[i].GetTireForce()
            wheel_states[i] = vehicle.GetWheelState(veh.WheelID(i))

        # Update modules (process inputs from other modules)
        time = vehicle.GetSystem().GetChTime()
        driver.Synchronize(time)
        powertrain.Synchronize(time, throttle_input, driveshaft_speed)
        vehicle.Synchronize(time, steering_input, braking_input, powertrain_torque, tire_forces)
        terrain.Synchronize(time)
        for i in range (num_wheels):
            tires[i].Synchronize(time, wheel_states[i], terrain)
        app.Synchronize(driver.GetInputModeAsString(), steering_input, throttle_input, braking_input)

        # Advance simulation for one timestep for all modules
        step = realtime_timer.SuggestSimulationStep(step_size)
        driver.Advance(step)
        powertrain.Advance(step)
        vehicle.Advance(step)
        terrain.Advance(step)
        for i in range(num_wheels) :
            tires[i].Advance(step)
        app.Advance(step)

        # Increment frame number
        step_number += 1

    return 0


# change these paths
vehicle_file =   "./data/vehicle/WheeledVehicle.json"
rigidtire_file = "./data/vehicle/RigidTire.json"
simplepowertrain_file = "vehicle/SimplePowertrain.json"

CHRONO_DATA_DIR = "C:/codes/Chrono/Chrono_Source/data/"

rigidterrain_file = "terrain/RigidPlane.json"

povray_output = False
# Initial vehicle position and orientation
initLoc = chrono.ChVectorD(0, 0, 1.0)
initRot = chrono.ChQuaternionD(1, 0, 0, 0)

# Simulation step size
step_size = 2e-3

# Time interval between two render frames
render_step_size = 1.0 / 50 #  // FPS = 50

# Point on chassis tracked by the camera
trackPoint = chrono.ChVectorD(0.0, 0.0, 1.75)

# Contact method
NSC_SMC = chrono.ChMaterialSurface.NSC

# Output directories
out_dir = "../WHEELED_VEHICLE"
pov_dir = out_dir + "/POVRAY"

main()