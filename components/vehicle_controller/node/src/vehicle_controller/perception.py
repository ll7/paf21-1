
from typing import Dict

class AckermannSignalAdapter:
    def signal_to_ackermann(self, signal):
        return signal

class PerceptionPreprocessor:
    def proprocess_perception(self, perceptions: Dict[str, any]):
        return perceptions[0]

class PerceptionCache:
    last_perception = None

    def update(self, perception):
        self.last_perception = perception
