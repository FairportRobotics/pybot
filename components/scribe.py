from wpilib import DataLogManager
import wpilib.deployinfo
from wpiutil.log import (
    BooleanLogEntry,
    DoubleLogEntry,
    IntegerLogEntry,
    StringLogEntry,
)

import wpilib


class Scribe:
    log_network_tables: bool = True
    log_console_output: bool = True

    def setup(self):
        # Starts recording to data log
        DataLogManager.start()
        DataLogManager.logNetworkTables(self.log_network_tables)
        DataLogManager.logConsoleOutput(self.log_console_output)

        deploy_config = wpilib.deployinfo.getDeployData()

        if deploy_config is not None:
            metadata_map = {
                "Deploy Host": "deploy-host",
                "Deploy User": "deploy-user",
                "Deploy Date": "deploy-date",
                "Code Path": "code-path",
                "Git Hash": "git-hash",
                "Git Branch": "git-branch",
                "Git Description": "git-desc",
            }

            for log_key, config_key in metadata_map.items():
                value = deploy_config.get(config_key, False)
                if value:
                    DataLogManager.log(f"{log_key}: {value}")

    def execute(self):
        pass

    def log(self, message: str):
        DataLogManager.log(message)

    def stop(self):
        DataLogManager.stop()
