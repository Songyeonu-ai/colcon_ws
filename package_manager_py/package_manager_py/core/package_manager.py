"""
Package Configuration Manager
"""

from typing import Dict, Optional
from ..package_settings.package_defaults import PackageConfig, DEFAULT_PACKAGES
from ..package_settings import settings

package_count=settings.DEFAULT_PACKAGE_COUNT+1

class PackageConfigManager:
    """
    Manages package configurations
    
    Can load from:
    - ROS2 parameters (preferred)
    - Default configuration (fallback)
    """
    
    def __init__(self, ros_node=None):
        """
        Initialize package configuration manager
        
        Args:
            ros_node: ROS2 node instance for reading parameters (optional)
        """
        self.packages: Dict[int, PackageConfig] = {}
        self._ros_node = ros_node
        
        # Load configurations
        if ros_node is not None:
            self._load_from_ros_parameters()
        else:
            self._load_from_defaults()
    
    def _load_from_ros_parameters(self):
        """Load package configurations from ROS2 parameters"""
        if self._ros_node is None:
            return
        
        for i in range(1, package_count):  # Support up to 11 packages
            prefix = f'packages.package{i}.'
            
            try:
                name = self._ros_node.get_parameter(f'{prefix}name').value
                executable = self._ros_node.get_parameter(f'{prefix}executable').value
                pkg_type = self._ros_node.get_parameter(f'{prefix}type').value
                
                # Get optional parameters with defaults
                try:
                    description = self._ros_node.get_parameter(f'{prefix}description').value
                except:
                    description = ""
                
                try:
                    auto_start = self._ros_node.get_parameter(f'{prefix}auto_start').value
                except:
                    auto_start = False
                
                self.packages[i] = PackageConfig(
                    name=name,
                    executable=executable,
                    pkg_type=pkg_type,
                    description=description,
                    auto_start=auto_start
                )
            except Exception as e:
                # If parameter doesn't exist, use default
                if i - 1 < len(DEFAULT_PACKAGES):
                    self.packages[i] = DEFAULT_PACKAGES[i - 1]
    
    def _load_from_defaults(self):
        """Load package configurations from defaults"""
        for i, config in enumerate(DEFAULT_PACKAGES, start=1):
            self.packages[i] = config
    
    def get_package(self, index: int) -> Optional[PackageConfig]:
        """
        Get package configuration by index
        
        Args:
            index: Package index (1-11)
            
        Returns:
            PackageConfig or None if not found
        """
        return self.packages.get(index)
    
    def get_package_by_name(self, name: str) -> Optional[PackageConfig]:
        """
        Get package configuration by name
        
        Args:
            name: Package name
            
        Returns:
            PackageConfig or None if not found
        """
        for config in self.packages.values():
            if config.name == name:
                return config
        return None
    
    def get_all_packages(self) -> Dict[int, PackageConfig]:
        """Get all package configurations"""
        return self.packages.copy()
    
    def get_package_count(self) -> int:
        """Get total number of configured packages"""
        return len(self.packages)
    
    def get_auto_start_packages(self) -> Dict[int, PackageConfig]:
        """Get packages configured for auto-start"""
        return {
            index: config
            for index, config in self.packages.items()
            if config.auto_start
        }
