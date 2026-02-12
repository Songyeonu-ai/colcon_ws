# ROS2 Package Manager

A modular, reusable package manager for ROS2 applications with PyQt5 GUI.

## Features

- ✅ **Modular Architecture**: Clean separation of concerns (core, UI, config, utils)
- ✅ **Reusable Components**: Use individual modules in other projects
- ✅ **ROS2 Integration**: Manage ROS2 packages (launch/run)
- ✅ **Real-time Monitoring**: Live status updates for all packages
- ✅ **Configuration Flexibility**: Configure via ROS2 parameters or defaults
- ✅ **User-Friendly UI**: Intuitive PyQt5 interface

## Architecture

```
package_manager_py/
├── config/           # Configuration management
│   ├── settings.py           # Global settings
│   └── package_defaults.py   # Default package configurations
├── core/             # Core functionality
│   ├── ros_node.py           # ROS2 node
│   ├── process_manager.py    # QProcess lifecycle management
│   └── package_manager.py    # Package configuration management
├── ui/               # User interface
│   ├── main_window.py        # Main application window
│   └── widgets/              # Reusable UI widgets
│       └── package_control.py
└── utils/            # Utilities
    └── constants.py          # Shared constants
```

## Installation

### Prerequisites
- ROS2 (Humble or later)
- Python 3.8+
- PyQt5

### Build
```bash
cd ~/ros2_ws/src
git clone <repository_url> package_manager_py
cd ~/ros2_ws
colcon build --packages-select package_manager_py
source install/setup.bash
```

## Usage

### Basic Usage
```bash
ros2 run package_manager_py package_manager
```

### Configuration via ROS2 Parameters

Create a parameter file `config.yaml`:

```yaml
package_manager_py_node:
  ros__parameters:
    packages:
      package1:
        name: "my_package"
        executable: "my_launch_file.launch.py"
        type: "launch"
        description: "My Custom Package"
        auto_start: false
      package2:
        name: "another_package"
        executable: "my_node"
        type: "run"
        description: "Another Package"
        auto_start: false
```

Run with parameters:
```bash
ros2 run package_manager_py package_manager --ros-args --params-file config.yaml
```

## API Usage

### Using as a Library

```python
import rclpy
from package_manager_py import RosNode, MainWindow
from PyQt5.QtWidgets import QApplication

# Initialize ROS2
rclpy.init()

# Create node
ros_node = RosNode()

# Create GUI
app = QApplication([])
window = MainWindow(ros_node)
window.show()

# Run
app.exec_()
```

### Using Individual Components

```python
# Use only the process manager
from package_manager_py.core import ProcessManager

process_mgr = ProcessManager()
success, msg = process_mgr.start_process(
    "my_package",
    "ros2",
    ["launch", "my_package", "my_launch.py"]
)
```

```python
# Use only the package configuration
from package_manager_py.config import PackageConfig, PackageConfigManager

config = PackageConfig(
    name="test_pkg",
    executable="test.launch.py",
    pkg_type="launch"
)

config_mgr = PackageConfigManager()
# ... use config manager
```

## Customization

### Adding Custom Packages

Edit `config/package_defaults.py`:

```python
DEFAULT_PACKAGES = [
    PackageConfig(
        name='your_package',
        executable='your_launch.py',
        pkg_type='launch',
        description='Your package description',
        auto_start=False
    ),
    # ... more packages
]
```

### Changing UI Settings

Edit `config/settings.py`:

```python
WINDOW_TITLE = "My Custom Package Manager"
MAX_PACKAGES = 10  # Support more packages
PROCESS_STOP_TIMEOUT = 1000  # Longer timeout
```

### Creating Custom Widgets

```python
from package_manager_py.ui.widgets import PackageControlWidget

class MyCustomWidget(PackageControlWidget):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # Add custom functionality
```

## Development

### Running Tests
```bash
pytest
```

### Code Style
```bash
flake8 package_manager_py
```

## License

Apache License 2.0

## Contributing

Contributions are welcome! Please feel free to submit a Pull Request.

## Maintainer

- **Name**: YeonU
- **Email**: yeonu0070@kw.ac.kr
