import commands2
import ntcore


class NetworkTables(commands2.Subsystem):
    """This class is used to interact with the network tables subsystem.
    It gives a CRUD interface with the network tables. You can create,
    read, update and delete topics in the network tables using those methods.
    """

    def __init__(
        self, table: str = "SmartDashboard", topics_and_types: list = []
    ) -> None:
        """
        Initialize the network tables subsystem.
        """
        super().__init__()
        nti = ntcore.NetworkTableInstance.getDefault()
        self.table = nti.getTable(table)

        self.topic = {}
        for topic_name, topic_type in topics_and_types:
            self.create(topic_name, topic_type)

    def create(self, topic_name: str, topic_type: str) -> None:
        """Create a topic in the network tables

        :param topic_name: The name of the topic to create
        :param topic_type: The type of the topic to create
        """
        if topic_type == "boolean":
            topic = self.table.getBooleanTopic(topic_name)
        elif topic_type == "integer":
            topic = self.table.getIntegerTopic(topic_name)
        elif topic_type == "double":
            topic = self.table.getDoubleTopic(topic_name)
        elif topic_type == "string":
            topic = self.table.getStringTopic(topic_name)
        elif topic_type == "raw":
            topic = self.table.getRawTopic(topic_name)
        elif topic_type == "boolean_array":
            topic = self.table.getBooleanArrayTopic(topic_name)
        elif topic_type == "integer_array":
            topic = self.table.getIntegerArrayTopic(topic_name)
        elif topic_type == "double_array":
            topic = self.table.getDoubleArrayTopic(topic_name)
        else:
            raise ValueError(f"Unsupported topic type: {topic_type}")
        self.topic[topic_name] = topic.publish()

    def read(self, topic_name: str) -> None:
        """Get the value of a topic

        :param topic_name: The name of the topic to read
        :return: The value of the topic
        """
        topic = self.topic.get(topic_name, None)
        self._validate_topic(topic_name)
        return self.table.getEntry(topic_name).getValue().value()

    def update(self, topic_name: str, value) -> None:
        """Updates the network table topic with the value

        :param topic_name: The name of the topic to update
        :param value: The value to set the topic to
        """
        topic = self.topic.get(topic_name, None)
        self._validate_topic(topic_name)
        topic.set(value)

    def delete(self, topic_name: str) -> None:
        """Delete a topic from the network tables

        :param topic_name: The name of the topic to delete
        """
        topic = self.topic.get(topic_name, None)
        self._validate_topic(topic_name)
        self.table.delete(topic_name)

    def _validate_topic(self, topic_name: str) -> None:
        """Validate that the topic exists in the network tables

        :param topic_name: The name of the topic to validate
        """
        if topic_name not in self.topic:
            raise ValueError(f"Topic {topic_name} not found in network tables")
