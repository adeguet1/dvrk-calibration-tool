from __future__ import division, print_function
import time
import json
from cisstCommonPython import *
from cisstVectorPython import *
from cisstOSAbstractionPython import *
from cisstMultiTaskPython import *
from cisstParameterTypesPython import *

NAME = 'NDITracker'
CONFIG_FILENAME = ("/home/cnookal1/catkin_ws/src/cisst-saw/"
    "sawNDITracker/share/polaris-active-tools.json")
PORT = "/dev/ttyUSB0"
period = 0.01

manager = mtsManagerLocal.GetInstance()
manager.CreateAllAndWait(5.0)
manager.StartAllAndWait(5.0)

proxy = mtsComponentWithManagement('{}Proxy'.format(NAME))
manager.AddComponent(proxy)
proxy.CreateAndWait(5.0)
time.sleep(0.5)

services = proxy.GetManagerComponentServices()

# create an instance of the tracker
print('--> loading dynamic library')
result = services.Load('sawNDITracker')
assert result, 'Failed to load {} using component services'.format('sawNDITracker')

print('--> create tracker component')
args = mtsTaskPeriodicConstructorArg(NAME, period, False, 256)
result = services.ComponentCreate('mtsNDISerial', args)
assert result, 'Failed to create {} of type {}'.format(NAME, 'mtsNDISerial')

# Configure the component
print('--> configure tracker component')
component = manager.GetComponent(NAME)
component.Configure(CONFIG_FILENAME)

# create the main interface to the tracker
print('--> create python interface for tracker')
controller = proxy.AddInterfaceRequiredAndConnect((NAME, 'Controller'))

component.CreateAndWait(5.0)
component.StartAndWait(5.0)

# initialize controller
print ('--> connect tracker to port')
controller.Connect(PORT)
time.sleep(2.0)

# see if the device is actually connected
print('--> make the tracker beep to make sure everything is ok')
controller.Beep(1)
time.sleep(1.0)
controller.Beep(2)

# look for all tools
toolNames = controller.ToolNames()

# create and connect interface for each tool
print ('--> connect interfaces for all tools')
tools = {}
for toolName in toolNames:
    print('  -- found tool: ' + toolName)
    tools[toolName] = (proxy.AddInterfaceRequiredAndConnect((NAME, toolName), 5))

# enable tracking
print('--> enabling tracking and beep twice')
controller.ToggleTracking(True)
controller.Beep(2)

# display positions
while True:
    time.sleep(0.5)
    for toolName, toolInterface in tools.items():
        pose = toolInterface.GetPositionCartesian()
        if pose.GetValid():  # if visible
            print(toolName + ': ' + str(pose.Position().Translation()))
        else:
            print(toolName + ' is not visible')