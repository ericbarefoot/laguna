# Camera

Unified access to DSLR and networked Pi cameras through one `CameraManager`
façade, plus a locally-attached OpenCV path.

::: laguna.camera

::: laguna.camera.dslr

::: laguna.camera.network

::: laguna.camera.local

## Pi-side agent

Runs on each Raspberry Pi to serve capture requests — see the module
docstring for the deployment shape.

::: laguna.camera.agent

## USB/gvfs recovery

Releases USB cameras from GNOME's gvfs auto-mount before libgphoto2 access,
which otherwise contends for the device.

::: laguna.camera.gvfs
