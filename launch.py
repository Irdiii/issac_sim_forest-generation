"""
Forest Generation Launcher
Initializes Isaac Sim application and runs the forest generation logic.
"""

from isaacsim import SimulationApp

# Initialize SimulationApp FIRST before importing omni modules
# This must be done at the start of the script
simulation_app = SimulationApp({
    "headless": False,
    "physics/cudaDevice": 0,    # Force Physics to NVIDIA
    "renderer/multiGpu/enabled": False
})

import omni.isaac.core.utils.prims as prim_utils
from forest_generation import ForestEnvironment
import omni.timeline

# Configuration
AREA_SIZE = 50.0
NUM_TREES = 50
NUM_ROCKS = 30

def main():
    print("Isaac Sim Forest Generation Launcher")
    print("="*60)
    
    # Create forest environment instance
    # We pass the existing World instance or let it create one (which needs App to be running)
    forest = ForestEnvironment(
        area_size=AREA_SIZE,
        num_trees=NUM_TREES,
        num_rocks=NUM_ROCKS
    )
    
    # Build the environment
    forest.build_environment()
    
    # Reset the world to initialize physics
    print("Resetting world...")
    forest.world.reset()
    
    # Keep the simulation running
    print("\nSimulation is running. Close the window to exit.")
    
    while simulation_app.is_running():
        forest.world.step(render=True)
    
    # Cleanup
    simulation_app.close()

if __name__ == "__main__":
    main()
