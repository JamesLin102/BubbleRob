from coppeliasim_zmqremoteapi_client import RemoteAPIClient
from tqdm.rich import tqdm
import math
import numpy as np
import cv2

client = RemoteAPIClient()
sim = client.require('sim')
simUI = client.require('simUI')

sim.setStepping(True)
sim.startSimulation()

# Get object handles
bubbleRobBase = sim.getObject("/bubbleRob")
leftMotor = sim.getObject("/leftmotor")  
rightMotor = sim.getObject("/rightmotor")
noseSensor = sim.getObject("/Proximity_sensor")
visionSensor = sim.getObject("/Vision_sensor")
passiveVisionSensor_1 = sim.getObject("/passive_Vision_sensor_1")
passiveVisionSensor_2 = sim.getObject("/passive_Vision_sensor_2")

minMaxSpeed = [10 * math.pi / 180, 300 * math.pi / 180]  # Minimum and maximum speeds
backUntilTime = -1  # Forward or backward mode flag
robotCollection = sim.createCollection(0)
sim.addItemToCollection(robotCollection, sim.handle_tree, bubbleRobBase, 0)

# Graphics and interface settings
distanceSegment = sim.addDrawingObject(sim.drawing_lines, 4, 0, -1, 1, [0, 1, 0])
robotTrace = sim.addDrawingObject(sim.drawing_linestrip + sim.drawing_cyclic, 2, 0, -1, 200, [1, 1, 0])
graph = sim.getObject("/graph")  
distStream = sim.addGraphStream(graph, 'bubbleRob clearance', 'm', 0, [1, 0, 0])

xml = '<ui title="' + sim.getObjectAlias(bubbleRobBase, 1) + ' speed" closeable="false" resizeable="false" activate="false">'
xml += '<hslider minimum="0" maximum="100" on-change="speedChange_callback" id="1"/>'
xml += '<label text="" style="* {margin-left: 300px;}"></label></ui>'
ui = simUI.create(xml)

current_speed = (minMaxSpeed[0] + minMaxSpeed[1]) * 0.5
simUI.setSliderValue(ui, 1, 100 * (current_speed - minMaxSpeed[0]) / (minMaxSpeed[1] - minMaxSpeed[0]))

def simulate():
    global backUntilTime, current_speed
    sim_time = 30  # Simulation time in seconds
    pbar = tqdm(range(int(sim_time / 0.05)), desc="Simulating")  # Assuming simulation step is 0.05 seconds
    for _ in pbar:
        t = sim.getSimulationTime()
        pbar.set_description(f"Simulating (time: {t:.2f} [s])")

        # Vision
        raw_img, resolution = sim.getVisionSensorImg(visionSensor)
      
        # from CoppeliaSim image to OpenCV form
        color_img = np.frombuffer(raw_img, dtype=np.uint8).reshape(resolution[1], resolution[0], 3)
        color_img = color_img.astype(float)/255
        color_img[color_img < 0] += 1
        color_img *= 255
        color_img = np.fliplr(color_img)
        color_img = color_img.astype(np.uint8)
        color_img = cv2.cvtColor(color_img, cv2.COLOR_RGB2BGR)

        # Canny Edge Detection
        blurred = cv2.GaussianBlur(color_img, (3, 3), 0)
        gray = cv2.cvtColor(blurred, cv2.COLOR_BGR2GRAY)
        edge_output = cv2.Canny(gray, 60, 110, apertureSize=3)  

        # from OpenCV form image return to CoppeliaSim
        canny = cv2.cvtColor(edge_output, cv2.COLOR_GRAY2RGB)
        canny = np.fliplr(canny)
        canny_bytes = canny.flatten().tobytes()
        sim.setVisionSensorImg(passiveVisionSensor_1, canny_bytes)

        # Show the detection range
        color_img = cv2.cvtColor(color_img, cv2.COLOR_BGR2RGB)
        rectangle = cv2.rectangle(color_img, (100,100), (150,150), (0,255,0), 2)
        rectangle = np.fliplr(rectangle)
        rectangle_bytes = rectangle.flatten().tobytes()
        sim.setVisionSensorImg(passiveVisionSensor_2, rectangle_bytes)

        # Sensing
        result_dis, distData, *_ = sim.checkDistance(robotCollection, sim.handle_all)
        if result_dis > 0:
            sim.addDrawingObjectItem(distanceSegment, None)
            sim.addDrawingObjectItem(distanceSegment, distData)
            sim.setGraphStreamValue(graph, distStream, distData[6])
        p = sim.getObjectPosition(bubbleRobBase)
        sim.addDrawingObjectItem(robotTrace, p)
        
        # Actuation
        result_pro = sim.readProximitySensor(noseSensor)
        if result_pro[0] > 0:
            backUntilTime = sim.getSimulationTime() + 4
        if backUntilTime < sim.getSimulationTime():
            sim.setJointTargetVelocity(leftMotor, current_speed)
            sim.setJointTargetVelocity(rightMotor, current_speed)
        else:
            sim.setJointTargetVelocity(leftMotor, -current_speed / 2)
            sim.setJointTargetVelocity(rightMotor, -current_speed / 8)

        # Find the red cylinder
        judgement = color_img[100:150, 100:150, 0]
        if result_pro[0] > 0 and np.mean(judgement) > 180:
            pbar.close()
            break

        sim.step()  # Perform a single simulation step

    sim.stopSimulation()
    simUI.destroy(ui)

simulate()
