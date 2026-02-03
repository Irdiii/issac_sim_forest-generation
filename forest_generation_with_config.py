"""
Example script showing how to use custom configuration
for Isaac Sim 5.1 Forest Generation
"""

from isaacsim import SimulationApp
import configparser
import os

# Configuration file path
CONFIG_FILE = "config.ini"

# Load configuration if it exists
config = configparser.ConfigParser()
if os.path.exists(CONFIG_FILE):
    config.read(CONFIG_FILE)
    AREA_SIZE = config.getfloat('environment', 'area_size', fallback=50.0)
    NUM_TREES = config.getint('trees', 'num_trees', fallback=50)
    NUM_ROCKS = config.getint('rocks', 'num_rocks', fallback=30)
else:
    # Default values
    AREA_SIZE = 50.0
    NUM_TREES = 50
    NUM_ROCKS = 30

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from forest_generation import ForestEnvironment


def main():
    """Main function with configuration file support"""
    print("Isaac Sim 5.1 Forest Generation (with config file)")
    print("="*60)
    print(f"Configuration loaded from: {CONFIG_FILE if os.path.exists(CONFIG_FILE) else 'defaults'}")
    
    # Create forest environment with configured parameters
    forest = ForestEnvironment(
        area_size=AREA_SIZE,
        num_trees=NUM_TREES,
        num_rocks=NUM_ROCKS
    )
    
    # Build the environment
    forest.build_environment()
    
    # Reset the world to initialize physics
    forest.world.reset()
    
    # Keep the simulation running
    print("\nSimulation is running. Close the window to exit.")
    
    # Run the simulation
    while simulation_app.is_running():
        forest.world.step(render=True)
    
    # Cleanup
    simulation_app.close()


if __name__ == "__main__":
    main()
