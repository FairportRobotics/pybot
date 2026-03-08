import math
from ntcore import NetworkTableInstance
from typing import Tuple

class Limelight:
    # Inspired by https://github.com/FRC703/Robotpy-Limelight
    def setup(self):
        nt = NetworkTableInstance.getDefault().getTable("limelight")
        self.ta_entry = nt.getDoubleTopic("ta").subscribe(0.0)
        self.tl_entry = nt.getDoubleTopic("tl").subscribe(0.0)
        self.ts_entry = nt.getDoubleTopic("ts").subscribe(0.0)
        self.tshort_entry = nt.getDoubleTopic("tshort").subscribe(0.0)
        self.tlong_entry = nt.getDoubleTopic("tlong").subscribe(0.0)
        self.thor_entry = nt.getDoubleTopic("thor").subscribe(0.0)
        self.tvert_entry = nt.getDoubleTopic("tvert").subscribe(0.0)
        self.tx_entry = nt.getDoubleTopic("tx").subscribe(0.0)
        self.ty_entry = nt.getDoubleTopic("ty").subscribe(0.0)
        self.tv_entry = nt.getIntegerTopic("tv").subscribe(0)

    def execute(self):
        pass

    @property
    def valid_targets(self) -> bool:
        """
        Whether the camera has found a valid target

        Returns:
            Any valid targets?
        """
        return self.tv_entry.get() == 1

    @property
    def horizontal_offset(self) -> float:
        """
        Gives the horizontal offset from the crosshair to the target
        LL1: -27° - 27°
        LL2: -29.8° - 29.8°

        Returns:
            The horizontal offest from the crosshair to the target.

        """
        return self.tx_entry.get()

    @property
    def vertical_offset(self) -> float:
        """
        Gives the vertical offset from the crosshair to the target
        LL1: -20.5° - 20.5°
        LL2: -24.85° - 24.85°

        Returns:
            The vertical offset from the crosshair to the target
        """
        return self.ty_entry.get()

    @property
    def target_area(self) -> float:
        """
        How much of the image is being filled by the target

        Returns:
            0% - 100% of image
        """
        return self.ta_entry.get()

    @property
    def skew(self) -> float:
        """
        How much the target is skewed

        Returns:
            -90° - 0°
        """
        return self.ts_entry.get()

    @property
    def latency(self) -> float:
        """
        How much the pipeline contributes to the latency. Adds at least 11ms for image capture

        Returns:
            Latency contribution
        """
        return self.tl_entry.get()

    @property
    def bb_short(self) -> float:
        """
        Sidelength of the shortest side of the fitted bouding box (pixels)

        Returns:
            Shortest sidelength
        """
        return self.tshort_entry.get()

    @property
    def bb_long(self) -> float:
        """
        Sidelength of the longest side of the fitted bouding box (pixels)

        Returns:
            Longest sidelength
        """
        return self.tlong_entry.get()

    @property
    def bb_horizontal(self) -> float:
        """
        Horizontal sidelength of the rough bounding box (0 - 320 px)

        Returns:
            The horizontal sidelength
        """
        return self.thor_entry.get()

    @property
    def bb_vertical(self) -> float:
        """
        Vertical sidelength of the rough bounding box (0 - 320 px)

        Returns:
            The vertical sidelength
        """
        return self.tvert_entry.get()
    '''
    @property
    def bounding_box(self) -> Tuple[float, float]:
        return (self.bb_horizontal, self.bb_vertical)

    def camtran(self) -> Tuple[Tuple[float, float, float], Tuple[float, float, float]]:
        """
        Results of a 3D solution position, 6 numbers: Translation(x,y,z) Rotation(pitch, yaw, roll)
        Honestly I have no clue what this does yet without some testing.
        """
        return self.__nt.getNumber("camtran", 0)

    @property
    def crosshair_ax(self) -> float:
        """
        Get crosshair A's X position
        """
        return self.__nt.getNumber("cx0", 0)

    @property
    def crosshair_ay(self) -> float:
        """
        Get crosshair A's Y position
        """
        return self.__nt.getNumber("cy0", 0)

    @property
    def crosshair_bx(self) -> float:
        """
        Get crosshair B's X position
        """
        return self.__nt.getNumber("cx1", 0)

    @property
    def crosshair_by(self) -> float:
        """
        Get crosshair B's Y position
        """
        return self.__nt.getNumber("cy1", 0)

    def camera(self, camMode: CamMode) -> None:
        """
        Set the camera mode. You can set this to be driver operated or to run on a pipeline.

        Args:
            camMode: The camera mode to set
        """
        self._enabled = camMode
        self.__nt.putNumber(
            "camMode", camMode.value if isinstance(camMode, CamMode) else camMode
        )

    def light(self, status: LEDState) -> None:
        """
        Set the status of the limelight lights

        Args:
            status: The status to set the light to
        """
        self._light = status
        self.__nt.putNumber(
            "ledMode", status.value if isinstance(status, LEDState) else status
        )

    def pipeline(self, pipeline: int):
        """
        Sets the currently active pipeline

        Args:
            pipeline: The pipeline id to set to be active
        """
        self._active_pipeline = 0
        self.__nt.putNumber("pipeline", pipeline)

    def snapshot(self, snapshotMode: SnapshotMode):
        """
        Allow users to take snapshots during a match

        Args:
            snapshotMode: The state to put the camera in
        """
        self._snapshots = snapshotMode
        self.__nt.putNumber(
            "snapshot",
            snapshotMode.value
            if isinstance(snapshotMode, SnapshotMode)
            else snapshotMode,
        )
    '''

    def calc_distance(self, camera_angle: float, mount_height: float, target_height: float):
        """
        Calculate the distance from the camera to the wall the target is mounted on

        Args:
            camera_angle: The angle of the camera, either known or calculated from the calc_camera_angle method
            mount_height: The height that the camera is mounted off the floor
            target_height:The height the target is from the floor

        Returns:
            Gives the distance (in the same units that were used for the input) away from the wall that has the target
        """
        return (target_height - mount_height) / math.tan(math.radians(camera_angle + self.vertical_offset))

    def calc_camera_angle(self, x_distance: float, mount_height: float, target_height: float):
        """
        Calculate the camera's mounted angle from known properties. Set the robot to a fixed
        distance away from the target and pass in the other properties and it will calculate
        the angle to put into the calc_distance function

        Args:
            x_distance: The known distance away from the wall the target is on
            mount_height: The height that the camera is mounted off the floor
            target_height: The height the target is from the floor

        Returns:
            Gives the angle (in degrees) that the camera is mounted at
        """
        return  -self.vertical_offset + math.degrees(math.atan((target_height - mount_height) / x_distance))