# Python API reference

These pages are generated statically from the Python source by mkdocstrings and
Griffe. Documentation builds do not import the native extension or any vendor SDK.

## Site lifecycle

::: waddle_sdk.site.Site

::: waddle_sdk.site.SiteSession

::: waddle_sdk.site.Run

::: waddle_sdk.site.load_site

## Runtime contracts

::: waddle_sdk.runtime
    options:
      members:
        - FaultCode
        - RuntimeFaultCause
        - RuntimeFault
        - RuntimeEvent
        - PartObservation
        - Observation
        - SubmitResult
        - RunPort
        - SdkRuntimePort
        - SupportFact
        - SupportRow
        - SupportMatrix
        - SdkSupportPort
        - SdkKinematicsPort
        - SdkGeometryPort

## Robot extension contracts

::: waddle_sdk.robots.site.PartConfig

::: waddle_sdk.robots.base.Driver

::: waddle_sdk.robots.base.PositionVelocityDriver

::: waddle_sdk.robots.base.CollisionSphere

::: waddle_sdk.robots.base.Arm

::: waddle_sdk.robots.base.Rig

## Camera extension contracts

::: waddle_sdk.cameras.site.CameraMount

::: waddle_sdk.cameras.site.CameraConfig

::: waddle_sdk.cameras.base.CameraFrame

::: waddle_sdk.cameras.base.CameraDriver

::: waddle_sdk.cameras.base.CameraCalibrationDriver

## Camera-only inspection

::: waddle_sdk.cameras.inspection.CameraInspectionSpec

::: waddle_sdk.cameras.inspection.CameraInspectionFrame

::: waddle_sdk.cameras.inspection.CameraInspection

::: waddle_sdk.cameras.inspection.CameraInspectionSession

::: waddle_sdk.cameras.inspection.CameraInspectionError

::: waddle_sdk.cameras.inspection.inspect_cameras

## Simulation extension contracts

::: waddle_sdk.simulation
    options:
      members:
        - WorldConfig
        - SimulationBackend
        - SimulationPartBackend
        - SimulationCameraBackend
        - SimulationFactoryError
        - resolve_simulation_factory
        - build_simulation_backend
        - simulation_backend_names

::: waddle_sdk.scene
    options:
      members:
        - Scene
        - SceneConfig
        - SceneArtifacts
        - CompiledScene
        - SimulationSceneCompiler
        - SceneError
        - SceneSyntaxError
        - SceneValidationError
        - ScenePathError
        - SceneCompilerError
        - load_scene
        - initialize_scene
        - resolve_scene_compiler
        - simulation_compiler_names

## Built-in simulation backends

::: waddle_sdk.robots.mujoco.MujocoBackend

::: waddle_sdk.robots.ros2.Ros2Backend

::: waddle_sdk.robots.ros2.Ros2Driver

::: waddle_sdk.robots.ros2.Ros2CameraDriver


## Non-opening hardware metadata

::: waddle_sdk.robots.metadata

::: waddle_sdk.cameras.metadata

::: waddle_sdk.robots.yam.model_sources

::: waddle_sdk.robots.models

## Site ownership errors

::: waddle_sdk.ownership.SiteOwnershipError
