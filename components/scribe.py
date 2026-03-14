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
    def setup(self):
        # Starts recording to data log
        DataLogManager.start()
        # Set up custom log entries
        l = DataLogManager.getLog()
        self.log = {
            "boolean": BooleanLogEntry(l, "/scribe/boolean"),
            "double": DoubleLogEntry(l, "/scribe/double"),
            "string": StringLogEntry(l, "/scribe/string"),
            "integer": IntegerLogEntry(l, "/scribe/integer"),
        }

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
                value = deploy_config.get(config_key, "")
                if value:
                    self.log["string"].append(f"{log_key}: {value}")

    def execute(self):
        pass
