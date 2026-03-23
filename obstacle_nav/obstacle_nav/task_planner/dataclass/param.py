import yaml
from dataclasses import dataclass, field 

def load_params(config_path):
    with open(config_path, 'r') as file:
        config_data = yaml.safe_load(file)
    return config_data

@dataclass #위쪽에 있는 파라미터들이 기본, 아래쪽은 세분화
class velocity_params:
    forward_speed: float = 0.0
    lateral_speed: float = 0.0
    angular_speed: float = 0.0

@dataclass
class geometry_params:
    distance: float = 0.0
    lane_width: float = 0.0
    theta: float = 0.0

@dataclass
class avoidance:
    geometry_params = field(default_factory=geometry_params)
    velocity_params = field(default_factory=velocity_params)

@dataclass
class detection:
    geometry_params = field(default_factory=geometry_params)
    geometry_params.distance = 0.5
    geometry_params.theta = 30.0
    min_ratio: float = 0.0



# 내가 중심적으로 튜닝할 값: 걷는 속도(앞, 옆, 돌기) / 장애물 인식(거리, 각도, 비율) / 
# 보조적으로 튜닝: 장애물 회피 시 목표 차선 결정 기준(차선 막힘 여부, 각도 기반 선호, 기본 선호)