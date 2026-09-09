"""Native GPU state for reference scenes with fixed-root articulations.

Follow SAPIEN/ManiSkill's build, initialize, apply and fetch lifecycle. Physics
and constraints remain entirely native; observations retain native velocities.
"""

from __future__ import annotations

from pathlib import Path


class State:
    def __init__(self):
        import sapien

        try:
            import torch
        except ImportError as error:
            raise RuntimeError(
                "SAPIEN bottle_cap requires CUDA and waddle-sdk[sapien-gpu] "
                "in the simulation worker's Python environment"
            ) from error
        if not torch.cuda.is_available():
            raise RuntimeError("SAPIEN bottle_cap requires an available CUDA device")
        library = (
            Path.home()
            / ".sapien"
            / "physx"
            / sapien.physx.version()
            / "libPhysXGpu_64.so"
        )
        if not sapien.physx.is_gpu_enabled() and not library.is_file():
            raise RuntimeError(
                "Install SAPIEN's GPU library before opening a bottle scene: "
                "run `python -c 'import sapien; sapien.physx.enable_gpu()'` "
                "in the simulation worker environment"
            )
        self.torch = torch
        sapien.physx.enable_gpu()
        self.physics = sapien.physx.PhysxGpuSystem()
        self.physics.gpu_set_cuda_stream(torch.cuda.current_stream().cuda_stream)

    def initialize(self, robot):
        self.physics.gpu_init()
        self.index = robot.gpu_index
        self.dof = len(robot.get_active_joints())
        self.q = self.physics.cuda_articulation_qpos.torch()
        self.dq = self.physics.cuda_articulation_qvel.torch()
        self.target = self.physics.cuda_articulation_target_qpos.torch()
        self.target_velocity = self.physics.cuda_articulation_target_qvel.torch()
        self.bodies = self.physics.cuda_rigid_dynamic_data.torch()
        self.read()

    def _assign(self, buffer, value):
        buffer[self.index, : self.dof] = self.torch.as_tensor(
            value, device=buffer.device, dtype=buffer.dtype
        )

    def read(self):
        self.physics.gpu_fetch_articulation_qpos()
        self.physics.gpu_fetch_articulation_qvel()
        return (
            self.q[self.index, : self.dof].cpu().numpy().copy(),
            self.dq[self.index, : self.dof].cpu().numpy().copy(),
        )

    def write(self, position, velocity):
        self._assign(self.target, position)
        self._assign(self.target_velocity, velocity)
        self.physics.gpu_apply_articulation_target_position()
        self.physics.gpu_apply_articulation_target_velocity()

    def home(self, position):
        self._assign(self.q, position)
        self._assign(self.dq, 0.0)
        self.physics.gpu_apply_articulation_qpos()
        self.physics.gpu_apply_articulation_qvel()
        self.physics.gpu_update_articulation_kinematics()

    def save(self):
        self.read()
        self.physics.gpu_fetch_rigid_dynamic_data()
        self.initial = tuple(value.clone() for value in (self.bodies, self.q, self.dq))

    def reset(self):
        for buffer, initial in zip(
            (self.bodies, self.q, self.dq), self.initial, strict=True
        ):
            buffer.copy_(initial)
        self.physics.gpu_apply_rigid_dynamic_data()
        self.physics.gpu_apply_articulation_qpos()
        self.physics.gpu_apply_articulation_qvel()
        self.physics.gpu_update_articulation_kinematics()

    def sync(self):
        self.physics.sync_poses_gpu_to_cpu()
