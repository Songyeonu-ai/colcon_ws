import math

def hz_to_ms(hz:float) -> float:
    if hz <= 0.0:
        raise ValueError("Frequency must be positive.")
    return 1000.0 / hz