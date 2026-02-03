# Isaac Sim Forest Generation

This project generates a forest environment with rocky terrain for NVIDIA Isaac Sim 5.1.

## Features

- **Configurable Area Size**: Create environments of any size (default: 50x50 meters)
- **Rocky Terrain**: Randomly generated rocks distributed across the area
- **Tree Generation**: Procedurally generated trees with varying heights and sizes
- **Physics-Enabled**: All objects have collision detection and physics properties
- **Lighting and Camera**: Pre-configured lighting and camera setup

## Requirements

- NVIDIA Isaac Sim 5.1 or later
- Python 3.7+
- NumPy

## Installation

1. Install [NVIDIA Isaac Sim 5.1](https://developer.nvidia.com/isaac-sim) following the official documentation

2. Clone this repository:
```bash
git clone https://github.com/Irdiii/issac_sim_forest-generation.git
cd issac_sim_forest-generation
```

## Usage

### Basic Usage

Run the forest generation script using Isaac Sim's Python:

```bash
# Linux
~/.local/share/ov/pkg/isaac_sim-5.1.0/python.sh forest_generation.py

# Windows
%USERPROFILE%\AppData\Local\ov\pkg\isaac_sim-5.1.0\python.bat forest_generation.py
```

### Configuration

You can modify the following parameters at the top of `forest_generation.py`:

```python
AREA_SIZE = 50.0    # Area size in meters (creates AREA_SIZE x AREA_SIZE area)
NUM_TREES = 50      # Number of trees to generate
NUM_ROCKS = 30      # Number of rocks for rocky terrain
```

### Advanced Configuration

For more control, you can also create a custom environment programmatically:

```python
from forest_generation import ForestEnvironment

# Create a custom forest
forest = ForestEnvironment(
    area_size=100.0,   # 100m x 100m area
    num_trees=100,     # 100 trees
    num_rocks=50       # 50 rocks
)

forest.build_environment()
```

## Environment Details

### Area
- Default: 50m x 50m (configurable)
- Ground plane with physics material
- Proper scaling and units

### Trees
- Procedurally generated using cylinders (trunk) and cones (foliage)
- Random heights: 3-8 meters
- Random trunk radius: 0.15-0.3 meters
- Random foliage size: 1.5-3.0 meters
- Physics collision enabled

### Rocky Terrain
- Random rock placement across the area
- Mix of sphere and cube shapes
- Random sizes: 0.5-2.5 meters
- Random heights: 0.3-1.5 meters
- Physics collision enabled

### Lighting
- Distant light (sun) at 45-degree angle
- Intensity: 3000 units

### Camera
- Positioned to view the entire area
- Angled at 30 degrees looking down
- Distance scales with area size

## Project Structure

```
issac_sim_forest-generation/
├── forest_generation.py    # Main script for forest generation
├── requirements.txt         # Python dependencies
└── README.md               # This file
```

## Troubleshooting

### Isaac Sim Not Found
Make sure Isaac Sim is properly installed and you're using the correct Python executable path.

### Import Errors
Ensure you're running the script with Isaac Sim's Python interpreter, not the system Python.

### Performance Issues
If the simulation runs slowly, try reducing `NUM_TREES` and `NUM_ROCKS` values.

## License

This project is open source and available under the MIT License.

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Acknowledgments

- NVIDIA Isaac Sim team for the simulation platform
- Forest generation algorithms and physics simulation