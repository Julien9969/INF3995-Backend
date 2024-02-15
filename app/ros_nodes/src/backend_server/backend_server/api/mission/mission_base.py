



# TODO Logigique de base modifiable au besoin 
class MissionBase:
    """Singleton mission base class."""
    mission = None

    @staticmethod
    def get_mission():
        """Get mission object."""
        if MissionBase.mission is None:
            MissionBase.mission = Mission()
        return MissionBase.mission
    
    @staticmethod
    def start_mission():
        """Start mission."""
        mission = MissionBase.get_mission()
        mission.start()
        print("Mission started")

    @staticmethod
    def stop_mission():
        """Stop mission."""
        mission = MissionBase.get_mission()
        mission.stop()
        print("Mission stopped")


class Mission:
    """Mission class."""
    def __init__(self):
        """Initialize mission."""
        self.running = False

    def start(self):
        """Start mission."""
        self.running = True

    def stop(self):
        """Stop mission."""
        self.running = False

    def __del__(self):
        """Delete mission."""
        self.stop()
        del self


