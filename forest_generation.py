"""
Isaac Sim 5.1 Forest Generation Script
Creates a configurable area with rocky terrain and trees
"""

from isaacsim import SimulationApp

# Configuration
AREA_SIZE = 50.0  # Area size in meters (50x50 by default)
NUM_TREES = 50  # Number of trees to generate
NUM_ROCKS = 30  # Number of rocks for rocky terrain

# Launch Isaac Sim
simulation_app = SimulationApp({"headless": False})

from omni.isaac.core import World
from omni.isaac.core.objects import DynamicCuboid
from omni.isaac.core.prims import XFormPrim
import omni.kit.commands
import omni.usd
from pxr import Gf, UsdGeom, UsdLux, Sdf
import numpy as np
import random


class ForestEnvironment:
    """
    Class to create a forest environment with rocky terrain
    """
    
    def __init__(self, area_size=50.0, num_trees=50, num_rocks=30):
        """
        Initialize the forest environment
        
        Args:
            area_size: Size of the area in meters (creates area_size x area_size)
            num_trees: Number of trees to generate
            num_rocks: Number of rocks to generate
        """
        self.area_size = area_size
        self.num_trees = num_trees
        self.num_rocks = num_rocks
        self.world = World(stage_units_in_meters=1.0)
        self.stage = omni.usd.get_context().get_stage()
        
    def create_ground_plane(self):
        """Create a ground plane for the terrain"""
        # Create ground plane
        plane_path = "/World/GroundPlane"
        omni.kit.commands.execute(
            "CreateMeshPrimWithDefaultXform",
            prim_type="Plane",
            prim_path=plane_path,
        )
        
        # Scale the ground plane to match area size
        plane_prim = self.stage.GetPrimAtPath(plane_path)
        xform_api = UsdGeom.XformCommonAPI(plane_prim)
        xform_api.SetScale((self.area_size / 2, 1.0, self.area_size / 2))
        
        # Add physics material
        omni.kit.commands.execute(
            "AddPhysicsMaterial",
            stage=self.stage,
            prim_path=Sdf.Path(plane_path),
            static_friction=0.6,
            dynamic_friction=0.5,
            restitution=0.3
        )
        
        print(f"Ground plane created: {self.area_size}m x {self.area_size}m")
        
    def create_rocky_terrain(self):
        """Generate rocky terrain with randomly placed rocks"""
        print(f"Generating {self.num_rocks} rocks...")
        
        for i in range(self.num_rocks):
            # Random position within the area
            x = random.uniform(-self.area_size/2, self.area_size/2)
            z = random.uniform(-self.area_size/2, self.area_size/2)
            
            # Random size for rocks
            rock_size = random.uniform(0.5, 2.5)
            height = random.uniform(0.3, 1.5)
            
            # Random rotation
            rotation_y = random.uniform(0, 360)
            
            # Create rock path
            rock_path = f"/World/Rocks/Rock_{i}"
            
            # Create a rock using a sphere or cube
            if random.random() > 0.5:
                # Create sphere rock
                omni.kit.commands.execute(
                    "CreateMeshPrimWithDefaultXform",
                    prim_type="Sphere",
                    prim_path=rock_path,
                )
                rock_prim = self.stage.GetPrimAtPath(rock_path)
                xform_api = UsdGeom.XformCommonAPI(rock_prim)
                xform_api.SetScale((rock_size, height, rock_size))
            else:
                # Create cube rock
                omni.kit.commands.execute(
                    "CreateMeshPrimWithDefaultXform",
                    prim_type="Cube",
                    prim_path=rock_path,
                )
                rock_prim = self.stage.GetPrimAtPath(rock_path)
                xform_api = UsdGeom.XformCommonAPI(rock_prim)
                xform_api.SetScale((rock_size, height, rock_size * 0.8))
            
            # Set position
            xform_api.SetTranslate((x, height/2, z))
            xform_api.SetRotate((0, rotation_y, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
            
            # Add collider
            omni.kit.commands.execute(
                "AddPhysicsCollider",
                stage=self.stage,
                prim_path=Sdf.Path(rock_path)
            )
            
        print(f"Rocky terrain created with {self.num_rocks} rocks")
        
    def create_tree(self, x, z, index):
        """
        Create a simple tree using cylinders and cones
        
        Args:
            x: X position
            z: Z position
            index: Tree index for naming
        """
        tree_height = random.uniform(3.0, 8.0)
        trunk_radius = random.uniform(0.15, 0.3)
        foliage_size = random.uniform(1.5, 3.0)
        
        # Create tree trunk
        trunk_path = f"/World/Trees/Tree_{index}/Trunk"
        omni.kit.commands.execute(
            "CreateMeshPrimWithDefaultXform",
            prim_type="Cylinder",
            prim_path=trunk_path,
        )
        
        trunk_prim = self.stage.GetPrimAtPath(trunk_path)
        trunk_xform = UsdGeom.XformCommonAPI(trunk_prim)
        trunk_xform.SetScale((trunk_radius, tree_height/2, trunk_radius))
        trunk_xform.SetTranslate((x, tree_height/2, z))
        
        # Add collider to trunk
        omni.kit.commands.execute(
            "AddPhysicsCollider",
            stage=self.stage,
            prim_path=Sdf.Path(trunk_path)
        )
        
        # Create foliage (cone)
        foliage_path = f"/World/Trees/Tree_{index}/Foliage"
        omni.kit.commands.execute(
            "CreateMeshPrimWithDefaultXform",
            prim_type="Cone",
            prim_path=foliage_path,
        )
        
        foliage_prim = self.stage.GetPrimAtPath(foliage_path)
        foliage_xform = UsdGeom.XformCommonAPI(foliage_prim)
        foliage_xform.SetScale((foliage_size, foliage_size, foliage_size))
        foliage_xform.SetTranslate((x, tree_height + foliage_size/2, z))
        
    def generate_trees(self):
        """Generate trees distributed across the area"""
        print(f"Generating {self.num_trees} trees...")
        
        for i in range(self.num_trees):
            # Random position within the area, with some margin
            margin = 2.0
            x = random.uniform(-self.area_size/2 + margin, self.area_size/2 - margin)
            z = random.uniform(-self.area_size/2 + margin, self.area_size/2 - margin)
            
            self.create_tree(x, z, i)
            
        print(f"Generated {self.num_trees} trees")
        
    def add_lighting(self):
        """Add lighting to the scene"""
        # Create a distant light (sun)
        light_path = "/World/SunLight"
        omni.kit.commands.execute(
            "CreatePrim",
            prim_path=light_path,
            prim_type="DistantLight",
        )
        
        light_prim = self.stage.GetPrimAtPath(light_path)
        light = UsdLux.DistantLight(light_prim)
        light.CreateIntensityAttr(3000.0)
        
        xform_api = UsdGeom.XformCommonAPI(light_prim)
        xform_api.SetRotate((45, 45, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        
        print("Lighting added")
        
    def setup_camera(self):
        """Setup a camera with a good view of the forest"""
        camera_path = "/World/Camera"
        omni.kit.commands.execute(
            "CreatePrim",
            prim_path=camera_path,
            prim_type="Camera",
        )
        
        camera_prim = self.stage.GetPrimAtPath(camera_path)
        xform_api = UsdGeom.XformCommonAPI(camera_prim)
        
        # Position camera to view the entire area
        camera_height = self.area_size * 0.6
        camera_distance = self.area_size * 0.8
        xform_api.SetTranslate((camera_distance, camera_height, camera_distance))
        xform_api.SetRotate((-30, 45, 0), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        
        print("Camera setup complete")
        
    def build_environment(self):
        """Build the complete forest environment"""
        print(f"\nBuilding forest environment: {self.area_size}m x {self.area_size}m")
        print("="*60)
        
        # Reset the world
        self.world.scene.add_default_ground_plane()
        
        # Create components
        self.create_ground_plane()
        self.create_rocky_terrain()
        self.generate_trees()
        self.add_lighting()
        self.setup_camera()
        
        print("="*60)
        print("Forest environment created successfully!")
        print(f"Area: {self.area_size}m x {self.area_size}m")
        print(f"Trees: {self.num_trees}")
        print(f"Rocks: {self.num_rocks}")
        print("="*60)


def main():
    """Main function to run the forest generation"""
    print("Isaac Sim 5.1 Forest Generation")
    print("="*60)
    
    # Create forest environment with configurable parameters
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
