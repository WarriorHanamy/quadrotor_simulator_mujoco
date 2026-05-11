"""Pydantic schema aligned with include/sim_schema.h."""

from pydantic import BaseModel, Field


# ---------------------------------------------------------------------------
# Physical limits (aligned with sim_schema.h + drone.xml)
# ---------------------------------------------------------------------------

GRAVITY_Z = -9.81  # [m/s²]
TIMESTEP = 0.001  # [s]
THRUST_MIN = 0.0  # [N]
THRUST_MAX = 42.0  # [N]
TORQUE_MIN = -0.5  # [Nm]
TORQUE_MAX = 0.5  # [Nm]
TOTAL_MASS = 1.0  # [kg]
IMAGE_MAX_W = 1280
IMAGE_MAX_H = 720
IMAGE_FOURCC = "rgb8"


# ---------------------------------------------------------------------------
# Control input
# ---------------------------------------------------------------------------


class QuadrotorControl(BaseModel):
    """Control command for the quadrotor simulator.

    Attributes set by external controllers (ROS, Python RL, etc.)
    and consumed by the core physics loop.
    """

    thrust: float = Field(
        default=0.0,
        ge=THRUST_MIN,
        le=THRUST_MAX,
        description="Body-frame Z thrust [N]",
    )
    torque: tuple[float, float, float] = Field(
        default=(0.0, 0.0, 0.0),
        description="Body-frame x/y/z moments [Nm], each in [-0.5, 0.5]",
    )


# ---------------------------------------------------------------------------
# State output
# ---------------------------------------------------------------------------


class QuadrotorState(BaseModel):
    """Full simulation state produced by the core physics loop."""

    time: float = Field(default=0.0, description="Simulation time [s]")
    position: tuple[float, float, float] = Field(
        default=(0.0, 0.0, 0.0),
        description="World-frame position x, y, z [m]",
    )
    orientation: tuple[float, float, float, float] = Field(
        default=(1.0, 0.0, 0.0, 0.0),
        description="World-frame quaternion w, x, y, z",
    )
    linear_velocity: tuple[float, float, float] = Field(
        default=(0.0, 0.0, 0.0),
        description="Body-frame linear velocity vx, vy, vz [m/s]",
    )
    angular_velocity: tuple[float, float, float] = Field(
        default=(0.0, 0.0, 0.0),
        description="Body-frame angular velocity wx, wy, wz [rad/s]",
    )
    linear_acceleration: tuple[float, float, float] = Field(
        default=(0.0, 0.0, 0.0),
        description="Body-frame linear acceleration ax, ay, az [m/s²]",
    )
