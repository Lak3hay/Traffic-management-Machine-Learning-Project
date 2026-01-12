import os
import sys

if "SUMO_HOME" not in os.environ:
    raise RuntimeError("SUMO_HOME is not set")

tools = os.path.join(os.environ["SUMO_HOME"], "tools")
sys.path.append(tools)

import traci
print("TraCI OK")
