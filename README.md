# SlimeVR Feeder Application

TODO

* It might be worth switching away from C++ because build systems/library management is a pain!
I'd use rust but rust's openvr stuff is... out of date and unmaintained.
maybe c#?

* Create default bindings for the boolean actions (possibly involving chording?)

* I think the choices made here regarding how to find/sort controllers is somewhat dubious, and out-of-date with slimevr.
The logic could use a re-do.

## How to use

You can download the feeder app by running the SlimeVR installer here: https://github.com/SlimeVR/SlimeVR-Installer/releases/latest/download/slimevr_web_installer.exe. This will make it launch automatically along with SteamVR.

Alternatively, you can download the feeder app here: https://github.com/SlimeVR/SlimeVR-Feeder-App/releases/latest/download/SlimeVR-Feeder-App-win64.zip and manually launch the .exe.

### Thanks
This was largely based off of https://github.com/Omnifinity/OpenVR-Tracking-Example , even if the structure is different. Thanks, @Omnifinity.

Rust setup was basically copy-pasted from https://github.com/XiangpengHao/cxx-cmake-example.
Thanks, @XiangpengHao.
