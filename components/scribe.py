import os
from pykit.wpilog.wpilogwriter import WPILOGWriter
from pykit.wpilog.wpilogreader import WPILOGReader
from pykit.networktables.nt4Publisher import NT4Publisher
from pykit.logger import Logger
import wpilib


class Scribe:
    LOGGER: Logger
    MODE: str

    def execute(self) -> None:
        """Standard MagicBot loop. Updates internal state."""
        pass

    def setup(self) -> None:
        """Called after injection but before the first loop."""
        self.LOGGER.recordMetadata("Robot", type(self).__name__)
        match self.MODE:
            case "REAL":
                deploy_config = wpilib.deployinfo.getDeployData()
                if deploy_config is not None:
                    self.LOGGER.recordMetadata(
                        "Deploy Host", deploy_config.get("deploy-host", "")
                    )
                    self.LOGGER.recordMetadata(
                        "Deploy User", deploy_config.get("deploy-user", "")
                    )
                    self.LOGGER.recordMetadata(
                        "Deploy Date", deploy_config.get("deploy-date", "")
                    )
                    self.LOGGER.recordMetadata(
                        "Code Path", deploy_config.get("code-path", "")
                    )
                    self.LOGGER.recordMetadata(
                        "Git Hash", deploy_config.get("git-hash", "")
                    )
                    self.LOGGER.recordMetadata(
                        "Git Branch", deploy_config.get("git-branch", "")
                    )
                    self.LOGGER.recordMetadata(
                        "Git Description", deploy_config.get("git-desc", "")
                    )
                self.LOGGER.addDataReciever(NT4Publisher(True))
                self.LOGGER.addDataReciever(WPILOGWriter())
            case "SIMULATION":
                self.LOGGER.addDataReciever(WPILOGWriter())
                self.LOGGER.addDataReciever(NT4Publisher(True))
            case "REPLAY":
                self.useTiming = (
                    False  # Disable timing in replay mode, run as fast as possible
                )
                log_path = os.environ["LOG_PATH"]
                log_path = os.path.abspath(log_path)
                print(f"Starting log from {log_path}")
                self.LOGGER.setReplaySource(WPILOGReader(log_path))
                self.LOGGER.addDataReciever(WPILOGWriter(log_path[:-7] + "_sim.wpilog"))
        self.LOGGER.start()

    def log(self, key: str, message: str) -> None:
        """Logs a message"""
        self.LOGGER.recordOutput(key, message)
