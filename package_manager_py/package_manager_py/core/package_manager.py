#설정 가져오는 역할만

from typing import Dict, Optional
from ..package_settings.package_defaults import PackageConfig, DEFAULT_PACKAGES
from ..package_settings import settings

package_count=settings.DEFAULT_PACKAGE_COUNT+1

class PackageConfigManager:
    def __init__(self, ros_node=None):
        self.packages: Dict[int, PackageConfig] = {}
        self._ros_node = ros_node
        
        if ros_node is not None:
            self._load_from_ros_parameters()
        else:
            self._load_from_defaults()
    
    def _load_from_ros_parameters(self):
        if self._ros_node is None:
            return
        
        for i in range(1, package_count):  #11 packages
            prefix = f'packages.package{i}.'
            
            try:
                name = self._ros_node.get_parameter(f'{prefix}name').value
                executable = self._ros_node.get_parameter(f'{prefix}executable').value
                pkg_type = self._ros_node.get_parameter(f'{prefix}type').value

                try:
                    description = self._ros_node.get_parameter(f'{prefix}description').value
                except:
                    description = ""
                
                try:
                    auto_start = self._ros_node.get_parameter(f'{prefix}auto_start').value
                except:
                    auto_start = False
                
                self.packages[i] = PackageConfig(
                    id=i,
                    name=name,
                    executable=executable,
                    pkg_type=pkg_type,
                    description=description,
                    auto_start=auto_start
                )
            except Exception as e:
                if i in DEFAULT_PACKAGES:
                    self.packages[i] = DEFAULT_PACKAGES[i]
    
    def _load_from_defaults(self):
        for pkg_id, config in DEFAULT_PACKAGES.items():
            self.packages[pkg_id] = config
    
    def get_package(self, index: int) -> Optional[PackageConfig]:
        return self.packages.get(index)
    
    # def get_package_by_name(self, name: str) -> Optional[PackageConfig]:
    #     for config in self.packages.values():
    #         if config.name == name:
    #             return config
    #     return None
    
    def get_all_packages(self) -> Dict[int, PackageConfig]:
        return self.packages.copy()
    
    def get_package_count(self) -> int:
        return len(self.packages)
    
    def get_auto_start_packages(self) -> Dict[int, PackageConfig]:
        return {
            index: config
            for index, config in self.packages.items()
            if config.auto_start
        }
