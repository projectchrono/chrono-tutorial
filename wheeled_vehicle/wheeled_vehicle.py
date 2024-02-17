import pychrono as chrono
import pychrono.vehicle as veh
import pychrono.irrlicht as chronoirr
import os
import math as m


def AddMovingObstacles(system) :
    # Create contact material, of appropriate type. Use default properties
    material = None
    if (NSC_SMC == chrono.ChContactMethod_NSC) :
        matNSC = chrono.ChContactMaterialNSC()
        #Change NSC material properties as desired
        material = matNSC
    elif (NSC_SMC == chrono.ChContactMethod_SMC) :
        matSMC = chrono.ChContactMaterialSMC()
        # Change SMC material properties as desired
        material = matSMC
    else:
        raise("Invalid Contact Method")

    sizeX = 300
    sizeY = 300
    height = 0
    numObstacles = 5

    for i in range(numObstacles) :
        o_sizeX = 1.0 + 3.0 * chrono.ChRandom.Get()
        o_sizeY = 0.3 + 0.4 * chrono.ChRandom.Get()
        o_sizeZ = 0.2 + 0.2 * chrono.ChRandom.Get()
        obstacle = chrono.ChBodyEasyBox(o_sizeX, o_sizeY, o_sizeZ, 2000.0, True, True, material)

        o_posX = chrono.ChRandom.Get() * 0.2 * sizeX
        o_posY = chrono.ChRandom.Get() * 0.2 * sizeY
        o_posZ = height + 4
        obstacle.SetPos(chrono.ChVector3d(o_posX, o_posY, o_posZ))
        obstacle.SetRot(chrono.QuatFromAngleX(chrono.CH_C_PI / 3) * chrono.QuatFromAngleY(chrono.CH_C_PI / 6))

        system.AddBody(obstacle)

def AddFixedObstacles(system) :
    # Create contact material, of appropriate type. Use default properties
    material = None
    if (NSC_SMC == chrono.ChContactMethod_NSC) :
        matNSC = chrono.ChContactMaterialNSC()
        #Change NSC material properties as desired
        material = matNSC
    elif (NSC_SMC == chrono.ChContactMethod_SMC) :
        matSMC = chrono.ChContactMaterialSMC()
        # Change SMC material properties as desired
        material = matSMC
    else:
        raise("Unvalid Contact Method")
    radius = 3;
    length = 10;
    obstacle = chrono.ChBodyEasyCylinder(chrono.ChAxis_Y, radius, length, 2000, True, True, material)

    obstacle.SetPos(chrono.ChVector3d(-20, 0, -2.7))
    obstacle.SetBodyFixed(True)

    system.AddBody(obstacle)

    for i in range(4) :
        stoneslab = chrono.ChBodyEasyBox(1.0, 5.0, 0.5, 2000, True, True, material)
        stoneslab.SetPos(chrono.ChVector3d(-1.2 * i + 22, -1.5, -0.25))
        
        stoneslab.SetRot(chrono.QuatFromAngleAxis(15 * chrono.CH_C_DEG_TO_RAD, chrono.VECT_Y))
        stoneslab.SetBodyFixed(True);
        system.AddBody(stoneslab);
        

def main():
    # ----------------------
    # Set path to data files
    # ----------------------

    # Path to Chrono data files (textures, etc.)
    print(CHRONO_DATA_DIR)
    chrono.SetChronoDataPath(CHRONO_DATA_DIR);

    # Path to the data files for this vehicle (JSON specification files)
    veh.SetDataPath("./data/")

    # --------------------------
    # Create the various modules
    # --------------------------
    
    # Create and initialize the vehicle system
    vehicle = veh.WheeledVehicle(veh.GetDataFile(vehicle_file), NSC_SMC)

    vehicle.Initialize(chrono.ChCoordsysd(initLoc, initRot))

    vehicle.SetSuspensionVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetSteeringVisualizationType(veh.VisualizationType_PRIMITIVES)
    vehicle.SetWheelVisualizationType(veh.VisualizationType_NONE)

    # Create the terrain
    terrain = veh.RigidTerrain(vehicle.GetSystem(), veh.GetDataFile(rigidterrain_file))
    terrain.Initialize()

    AddFixedObstacles(vehicle.GetSystem())
    AddMovingObstacles(vehicle.GetSystem())

    # Create and initialize the powertrain system
    engine = veh.ReadEngineJSON(veh.GetDataFile(engine_file))
    transmission = veh.ReadTransmissionJSON(veh.GetDataFile(transmission_file))
    powertrain = veh.ChPowertrainAssembly(engine, transmission)
    vehicle.InitializePowertrain(powertrain)

    # Create and initialize the tires
    for axle in vehicle.GetAxles() :
        tireL = veh.RigidTire(veh.GetDataFile(rigidtire_file))
        tireR = veh.RigidTire(veh.GetDataFile(rigidtire_file))
        vehicle.InitializeTire(tireL, axle.m_wheels[0], veh.VisualizationType_MESH)
        vehicle.InitializeTire(tireR, axle.m_wheels[1], veh.VisualizationType_MESH)

    # Set collision system
    vehicle.GetSystem().SetCollisionSystemType(chrono.ChCollisionSystem.Type_BULLET)

    # Create the Irrlicht vehicle application
    vis = veh.ChWheeledVehicleVisualSystemIrrlicht()
    vis.SetWindowTitle('Vehicle demo')
    vis.SetWindowSize(1280, 1024)
    vis.SetChaseCamera(chrono.ChVector3d(0.0, 0.0, 1.75), 6.0, 0.5)
    vis.Initialize()
    vis.AddLogo(chrono.GetChronoDataFile('logo_pychrono_alpha.png'))
    vis.AddLightDirectional()
    vis.AddSkyBox()
    vis.AttachVehicle(vehicle)

    # Create the driver system (interactive)
    driver = veh.ChInteractiveDriverIRR(vis)
    driver.SetSteeringDelta(0.02)
    driver.SetThrottleDelta(0.02)
    driver.SetBrakingDelta(0.02)
    driver.Initialize()

    # ---------------
    # Simulation loop
    # ---------------

    # Number of simulation steps between two 3D view render frames
    render_steps = m.ceil(render_step_size / step_size)

    #Initialize simulation frame counter and simulation time
    time = 0

    vehicle.EnableRealtime(True)

    while vis.Run() :
        # Render scene
        vis.BeginScene()
        vis.Render()
        vis.EndScene()

        # Get driver inputs
        driver_inputs = driver.GetInputs()

        # Update modules (process inputs from other modules)
        time = vehicle.GetSystem().GetChTime()
        driver.Synchronize(time)
        vehicle.Synchronize(time, driver_inputs, terrain)
        terrain.Synchronize(time)
        vis.Synchronize(time, driver_inputs)

        # Advance simulation for one timestep for all modules
        driver.Advance(step_size)
        vehicle.Advance(step_size)
        terrain.Advance(step_size)
        vis.Advance(step_size)

    return 0


# change these paths
vehicle_file =   "vehicle/WheeledVehicle.json"
rigidtire_file = "vehicle/RigidTire.json"
engine_file = "vehicle/EngineSimple.json"
transmission_file = "vehicle/AutomaticTransmissionSimpleMap.json"

CHRONO_DATA_DIR = "E:/Repositories/chrono/data/"

rigidterrain_file = "terrain/RigidPlane.json"

povray_output = False
# Initial vehicle position and orientation
initLoc = chrono.ChVector3d(0, 0, 1.0)
initRot = chrono.ChQuaterniond(1, 0, 0, 0)

# Simulation step size
step_size = 2e-3

# Time interval between two render frames
render_step_size = 1.0 / 50 #  // FPS = 50

# Point on chassis tracked by the camera
trackPoint = chrono.ChVector3d(0.0, 0.0, 1.75)

# Contact method
NSC_SMC = chrono.ChContactMethod_NSC

main()