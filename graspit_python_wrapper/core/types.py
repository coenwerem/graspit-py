"""
Core data types for graspit-py.
"""

from dataclasses import dataclass
from typing import List, Optional, Dict, Any
import numpy as np
from enum import Enum


class GraspType(Enum):
    """Types of grasps."""

    UNKNOWN = "unknown"
    PRECISION = "precision"
    POWER = "power"
    LATERAL = "lateral"
    CYLINDRICAL = "cylindrical"
    SPHERICAL = "spherical"


class QualityMetric(Enum):
    """Available quality metrics."""

    VOLUME = "volume"
    EPSILON = "epsilon"
    VOLUME_EPSILON = "volume_epsilon"
    MINIMUM_FORCE = "minimum_force"
    CONTACT_PATCH = "contact_patch"


@dataclass
class HandConfiguration:
    """Configuration of a robot hand."""

    joint_positions: np.ndarray
    joint_velocities: Optional[np.ndarray] = None
    grasp_type: Optional[GraspType] = None

    def __post_init__(self):
        self.joint_positions = np.array(self.joint_positions)
        if self.joint_velocities is not None:
            self.joint_velocities = np.array(self.joint_velocities)


@dataclass
class ObjectPose:
    """Pose of an object in the world."""

    position: np.ndarray
    orientation: np.ndarray  # Quaternion [x, y, z, w]

    def __post_init__(self):
        self.position = np.array(self.position)
        self.orientation = np.array(self.orientation)


@dataclass
class ContactPoint:
    """A contact point between hand and object."""

    position: np.ndarray
    normal: np.ndarray
    force: Optional[np.ndarray] = None

    def __post_init__(self):
        self.position = np.array(self.position)
        self.normal = np.array(self.normal)
        if self.force is not None:
            self.force = np.array(self.force)


@dataclass
class GraspQuality:
    """Quality metrics for a grasp."""

    volume: float
    epsilon: float
    volume_epsilon: float
    min_force: Optional[float] = None
    contact_patch: Optional[float] = None

    def to_dict(self) -> Dict[str, float]:
        """Convert to dictionary."""
        return {
            "volume": self.volume,
            "epsilon": self.epsilon,
            "volume_epsilon": self.volume_epsilon,
            "min_force": self.min_force,
            "contact_patch": self.contact_patch,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "GraspQuality":
        """Create from dictionary."""
        return cls(**data)


@dataclass
class GraspResult:
    """Complete grasp result."""

    hand_config: HandConfiguration
    object_pose: ObjectPose
    contacts: List[ContactPoint]
    quality: GraspQuality
    is_non_penetrating: bool
    grasp_type: GraspType
    metadata: Optional[Dict[str, Any]] = None

    def to_dict(self) -> Dict[str, Any]:
        """Convert to dictionary for serialization."""
        return {
            "hand_config": {
                "joint_positions": self.hand_config.joint_positions.tolist(),
                "joint_velocities": self.hand_config.joint_velocities.tolist()
                if self.hand_config.joint_velocities is not None
                else None,
                "grasp_type": self.hand_config.grasp_type.value
                if self.hand_config.grasp_type
                else None,
            },
            "object_pose": {
                "position": self.object_pose.position.tolist(),
                "orientation": self.object_pose.orientation.tolist(),
            },
            "contacts": [
                {
                    "position": contact.position.tolist(),
                    "normal": contact.normal.tolist(),
                    "force": contact.force.tolist()
                    if contact.force is not None
                    else None,
                }
                for contact in self.contacts
            ],
            "quality": self.quality.to_dict(),
            "is_non_penetrating": self.is_non_penetrating,
            "grasp_type": self.grasp_type.value,
            "metadata": self.metadata,
        }

    @classmethod
    def from_dict(cls, data: Dict[str, Any]) -> "GraspResult":
        """Create from dictionary."""
        hand_config = HandConfiguration(
            joint_positions=data["hand_config"]["joint_positions"],
            joint_velocities=data["hand_config"]["joint_velocities"],
            grasp_type=GraspType(data["hand_config"]["grasp_type"])
            if data["hand_config"]["grasp_type"]
            else None,
        )

        object_pose = ObjectPose(
            position=data["object_pose"]["position"],
            orientation=data["object_pose"]["orientation"],
        )

        contacts = [
            ContactPoint(
                position=contact_data["position"],
                normal=contact_data["normal"],
                force=contact_data["force"],
            )
            for contact_data in data["contacts"]
        ]

        quality = GraspQuality.from_dict(data["quality"])

        return cls(
            hand_config=hand_config,
            object_pose=object_pose,
            contacts=contacts,
            quality=quality,
            is_non_penetrating=data["is_non_penetrating"],
            grasp_type=GraspType(data["grasp_type"]),
            metadata=data.get("metadata"),
        )
