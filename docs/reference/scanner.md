# Scanner (Gocator)

Driver for the LMI Gocator 2690 laser line-profile 3D scanner. See the
[subsystem guide](../subsystems/scanner.md) for usage and
[scanner setup & tuning](../subsystems/scanner-setup.md) for hardware
bring-up.

## Scanner

::: laguna.scanner.gocator

## GoSDK bindings

::: laguna.scanner.gosdk

## Mounting / coordinate transform

::: laguna.scanner.mounting

## Point cloud utilities

::: laguna.scanner.pointcloud

## Profile utilities

Single-line (Profile mode) container and conversion — see the [subsystem
guide](../subsystems/scanner.md#profile-mode-a-single-line) for when to use
`scan_profile()` instead of a surface scan.

::: laguna.scanner.profile

## Settings

::: laguna.scanner.settings

## Simulation

A GoSdk stand-in returning synthetic surfaces, for `simulate=True` rehearsal
— see [Simulation](simulation.md) for the full-experiment version.

::: laguna.scanner.simulation
