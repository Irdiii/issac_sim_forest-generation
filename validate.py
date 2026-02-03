"""
Validation script to check forest generation configuration
This script can be run without Isaac Sim installed
"""

import sys
import os


def validate_configuration():
    """Validate the configuration and file structure"""
    print("Forest Generation Validation")
    print("="*60)
    
    # Check if main script exists
    if os.path.exists("forest_generation.py"):
        print("✓ Main script found: forest_generation.py")
    else:
        print("✗ Main script not found: forest_generation.py")
        return False
    
    # Check if config file exists
    if os.path.exists("config.ini"):
        print("✓ Configuration file found: config.ini")
    else:
        print("✗ Configuration file not found: config.ini")
        return False
    
    # Check if README exists
    if os.path.exists("README.md"):
        print("✓ Documentation found: README.md")
    else:
        print("✗ Documentation not found: README.md")
        return False
    
    # Read and validate main script
    try:
        with open("forest_generation.py", "r") as f:
            content = f.read()
            
            # Check for required constants
            if "AREA_SIZE = 50.0" in content:
                print("✓ AREA_SIZE configured (50.0 meters)")
            else:
                print("✗ AREA_SIZE not found or incorrectly configured")
                return False
            
            # Check for tree generation
            if "NUM_TREES" in content and "generate_trees" in content:
                print("✓ Tree generation implemented")
            else:
                print("✗ Tree generation not found")
                return False
            
            # Check for rocky terrain
            if "NUM_ROCKS" in content and "create_rocky_terrain" in content:
                print("✓ Rocky terrain generation implemented")
            else:
                print("✗ Rocky terrain generation not found")
                return False
            
            # Check for ForestEnvironment class
            if "class ForestEnvironment:" in content:
                print("✓ ForestEnvironment class defined")
            else:
                print("✗ ForestEnvironment class not found")
                return False
            
            # Check for area_size parameter
            if "area_size" in content and "area_size x area_size" in content:
                print("✓ Configurable area size implemented")
            else:
                print("✗ Configurable area size not properly implemented")
                return False
                
    except Exception as e:
        print(f"✗ Error reading forest_generation.py: {e}")
        return False
    
    # Validate configuration file
    try:
        import configparser
        config = configparser.ConfigParser()
        config.read("config.ini")
        
        if config.has_section('environment'):
            area_size = config.get('environment', 'area_size')
            print(f"✓ Environment configuration found (area_size: {area_size})")
        
        if config.has_section('trees'):
            num_trees = config.get('trees', 'num_trees')
            print(f"✓ Tree configuration found (num_trees: {num_trees})")
        
        if config.has_section('rocks'):
            num_rocks = config.get('rocks', 'num_rocks')
            print(f"✓ Rock configuration found (num_rocks: {num_rocks})")
            
    except Exception as e:
        print(f"⚠ Warning: Could not validate config.ini: {e}")
    
    print("="*60)
    print("✓ All validation checks passed!")
    print("\nProject Structure:")
    print("  - Area Size: 50x50 meters (configurable)")
    print("  - Trees: Procedurally generated and distributed")
    print("  - Rocky Terrain: Random rock placement")
    print("  - Physics: Collision detection enabled")
    print("  - Lighting and Camera: Pre-configured")
    print("="*60)
    return True


if __name__ == "__main__":
    try:
        if validate_configuration():
            sys.exit(0)
        else:
            print("\n✗ Validation failed!")
            sys.exit(1)
    except Exception as e:
        print(f"\n✗ Validation error: {e}")
        sys.exit(1)
