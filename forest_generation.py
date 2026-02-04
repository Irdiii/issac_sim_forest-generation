"""
Isaac Sim 5.1 Forest Generation Script
Creates a configurable area with rocky terrain and trees
"""

# SimulationApp initialization removed. 
# Use launch.py to run this environment independently.

# Imports
from omni.isaac.core import World
from omni.isaac.core.objects import FixedCuboid, FixedSphere, FixedCylinder, FixedCone
from omni.isaac.core.prims import XFormPrim
from omni.isaac.core.utils.prims import create_prim
import omni.kit.commands
import omni.usd
from pxr import Gf, UsdGeom, UsdLux, Sdf, UsdPhysics
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
            # Random position within the area (Z-up system: X and Y are ground plane)
            x = random.uniform(-self.area_size/2, self.area_size/2)
            y = random.uniform(-self.area_size/2, self.area_size/2)
            
            # Sample terrain height
            z_ground = self.get_terrain_height(x, y)
            
            # Random size for rocks
            rock_size = random.uniform(0.5, 2.5)
            height = random.uniform(0.3, 1.5)
            
            # Create parent Xform for the rock to handle positioning
            # Place at (x, y, z_ground)
            rock_xform_path = f"/World/Rocks/Rock_{i}"
            create_prim(rock_xform_path, "Xform")
            
            # Wrap in XFormPrim to enforce world pose reliably
            rock_parent = XFormPrim(rock_xform_path)
            rock_parent.set_world_pose(position=np.array([x, y, z_ground]))
            
            if i < 5:
                print(f"Rock {i}: World Pos=({x:.2f}, {y:.2f}, {z_ground:.2f})")
            
            # (Geometry creation same as before...)
            
            # Create a rock using a sphere or cube as a child of the Xform
            # Position is LOCAL to the Xform (offset by height/2 in Z)
            if random.random() > 0.5:
                # Create sphere rock
                rock = FixedSphere(
                    prim_path=f"{rock_xform_path}/Geometry",
                    name=f"rock_sphere_{i}",
                    scale=np.array([rock_size, rock_size, height]),
                    color=np.array([0.5, 0.5, 0.5])
                )
                rock.set_local_pose(translation=np.array([0, 0, height/2]))
            else:
                # Create cube rock
                rock = FixedCuboid(
                    prim_path=f"{rock_xform_path}/Geometry",
                    name=f"rock_cube_{i}",
                    scale=np.array([rock_size, rock_size * 0.8, height]),
                    color=np.array([0.4, 0.4, 0.4])
                )
                rock.set_local_pose(translation=np.array([0, 0, height/2]))
            
        print(f"Rocky terrain created with {self.num_rocks} rocks")
        
    def create_tree(self, x, y, index):
        """
        Create a simple tree using cylinders and cones
        
        Args:
            x: X position
            y: Y position (Ground)
            index: Tree index for naming
        """
        tree_height = random.uniform(3.0, 8.0)
        trunk_radius = random.uniform(0.15, 0.3)
        foliage_size = random.uniform(1.5, 3.0)
        
    def create_tree(self, x, y, index):
        """
        Create a simple tree using cylinders and cones
        
        Args:
            x: X position
            y: Y position (Ground)
            index: Tree index for naming
        """
        # Apply strict size randomization (1x to 1.5x global multiplier)
        size_multiplier = random.uniform(1.0, 1.5)
        
        # Base dimensions (scaled by multiplier)
        tree_height = random.uniform(3.0, 7.0) * size_multiplier
        trunk_radius = random.uniform(0.15, 0.3) * size_multiplier
        foliage_size = random.uniform(1.5, 3.0) * size_multiplier
        
        # Create parent Xform for the tree
        tree_xform_path = f"/World/Trees/Tree_{index}"
        create_prim(tree_xform_path, "Xform")
        
        # Sample terrain height for Z
        z_ground = self.get_terrain_height(x, y)
        
        # Wrap in XFormPrim to enforce world pose reliably
        tree_parent = XFormPrim(tree_xform_path)
        tree_parent.set_world_pose(position=np.array([x, y, z_ground]))
        
        if index < 5:
            print(f"Tree {index}: Pos=({x:.2f}, {y:.2f}, {z_ground:.2f}), SizeMult={size_multiplier:.2f}, H={tree_height:.2f}")
        
        # Create tree trunk (Child of Xform)
        # ASSUMPTION: FixedCylinder base height is 1.0. 
        # We scale Z by tree_height, so total height/length is tree_height.
        # Cylinder center is at (0,0,0). So extent is [-H/2, H/2].
        # We want Bottom at Z=0. So Center must be at Z = H/2.
        trunk_pos_z = tree_height / 2
        
        trunk = FixedCylinder(
            prim_path=f"{tree_xform_path}/Trunk",
            name=f"tree_trunk_{index}",
            scale=np.array([trunk_radius, trunk_radius, tree_height]),
            color=np.array([0.4, 0.26, 0.13]) # Brown
        )
        trunk.set_local_pose(translation=np.array([0, 0, trunk_pos_z]))
        
        # Create foliage (Child of Xform)
        # Foliage should sit exactly ON TOP of the trunk.
        # Foliage (Cone) Base Height likely 1.0. Scale = foliage_size.
        # Cone Center is at 0. Extent [-size/2, size/2].
        # To rest on trunk top (Z = tree_height):
        # Center = tree_height + (foliage_size / 2)
        foliage_pos_z = tree_height + (foliage_size / 2)
        
        foliage = FixedCone(
            prim_path=f"{tree_xform_path}/Foliage",
            name=f"tree_foliage_{index}",
            scale=np.array([foliage_size, foliage_size, foliage_size]),
            color=np.array([0.0, 0.4, 0.0]) # Green
        )
        foliage.set_local_pose(translation=np.array([0, 0, foliage_pos_z]))
        
    def generate_trees(self):
        """Generate trees distributed across the area"""
        print(f"Generating {self.num_trees} trees...")
        
        for i in range(self.num_trees):
            x = random.uniform(-self.area_size/2, self.area_size/2)
            y = random.uniform(-self.area_size/2, self.area_size/2)
            self.create_tree(x, y, i)
            
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
        
    def create_dome_light(self):
        """Create a dome light for global illumination"""
        light_path = "/World/DomeLight"
        omni.kit.commands.execute(
            "CreatePrim",
            prim_path=light_path,
            prim_type="DomeLight",
        )
        light_prim = self.stage.GetPrimAtPath(light_path)
        light = UsdLux.DomeLight(light_prim)
        light.CreateIntensityAttr(1000.0)
        print("Dome light added")
        
        print("Camera setup complete")
        
    def setup_camera(self):
        """Setup a camera with a good view of the forest"""
        import omni.kit.viewport.utility
        
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
        
        # Force the active viewport to use this camera
        print("Setting active viewport camera...")
        try:
            viewport = omni.kit.viewport.utility.get_active_viewport()
            if viewport:
                viewport.set_active_camera(camera_path)
                print(f"Active camera set to {camera_path}")
            else:
                print("No active viewport found to set camera")
        except Exception as e:
            print(f"Error setting active camera: {e}")
            
        print("Camera setup complete")
        
    def get_terrain_height(self, x, y):
        """
        Calculate terrain height at (x, y) using sine waves.
        """
        # Frequency and Amplitude for "bumpy" hills
        f1 = 0.15
        a1 = 1.0
        
        f2 = 0.4
        a2 = 0.3
        
        z = a1 * np.sin(f1 * x) * np.cos(f1 * y) + \
            a2 * np.sin(f2 * x) * np.cos(f2 * y)
            
        return z

    def create_terrain_mesh(self):
        """Create a procedural bumpy terrain mesh"""
        print("Generating terrain mesh...")
        stage = self.stage
        if not stage:
            return

        # Grid parameters
        size = self.area_size
        segments = 100 # Resolution
        step = size / segments
        
        points = []
        indices = []
        counts = [] # Face vertex counts (all 3 for triangles or 4 for quads)
        
        # 1. Generate Vertices
        # Centered at (0,0)
        start = -size / 2
        
        for i in range(segments + 1):
            x = start + i * step
            for j in range(segments + 1):
                y = start + j * step
                z = self.get_terrain_height(x, y)
                points.append((x, y, z))
                
        # 2. Generate Topology (Quads)
        # Grid of (segments x segments) cells
        # Vertex (i, j) index = i * (segments + 1) + j
        row_len = segments + 1
        
        for i in range(segments):
            for j in range(segments):
                # Indices of 4 corners
                # TL(i, j) -- TR(i+1, j)
                # |           |
                # BL(i, j+1)-- BR(i+1, j+1)
                
                # Note: Coordinate system
                # x grows with i, y grows with j
                
                idx_tl = i * row_len + j
                idx_bl = i * row_len + (j + 1)
                idx_tr = (i + 1) * row_len + j
                idx_br = (i + 1) * row_len + (j + 1)
                
                # Face 1 (Quad)
                # Order matters for normals (usually CCW)
                indices.extend([idx_tl, idx_tr, idx_br, idx_bl])
                counts.append(4)

        # 3. Create USD Mesh
        terrain_path = "/World/Terrain"
        mesh = UsdGeom.Mesh.Define(stage, terrain_path)
        
        # Set Data
        mesh.CreatePointsAttr(points)
        mesh.CreateFaceVertexIndicesAttr(indices)
        mesh.CreateFaceVertexCountsAttr(counts)
        
        # Add coloring
        mesh.CreateDisplayColorAttr([(0.2, 0.35, 0.2)]) # Dark Green
        
        # Add physics collision
        UsdPhysics.CollisionAPI.Apply(mesh.GetPrim())

    def build_environment(self):
        """Build the complete forest environment"""
        print(f"\nBuilding forest environment: {self.area_size}m x {self.area_size}m")
        print("="*60)

        
        # Create components
        self.create_terrain_mesh()
        self.create_rocky_terrain()
        self.generate_trees()
        self.add_lighting()
        self.create_dome_light()
        self.setup_camera()
        
        print("="*60)
        print("Forest environment created successfully!")
        print(f"Area: {self.area_size}m x {self.area_size}m")
        print(f"Trees: {self.num_trees}")
        print(f"Rocks: {self.num_rocks}")
        print("="*60)
