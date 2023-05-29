#include <openvr.h>
#include <fmt/core.h>
#include <memory>
#include "pathtools_excerpt.h"

static constexpr const char *rel_manifest_path = "./manifest.vrmanifest";
static constexpr const char *application_key = "slimevr.steamvr.feeder";

void shutdown_vr(vr::IVRSystem* _system) {
	vr::VR_Shutdown();
}

int handle_setup(bool install) {
    vr::EVRInitError init_error = vr::VRInitError_None;
    std::unique_ptr<vr::IVRSystem, decltype(&shutdown_vr)> system(vr::VR_Init(&init_error, vr::VRApplication_Utility), &shutdown_vr);

    if (init_error != vr::VRInitError_None) {
		fmt::print("Unable to init VR runtime as utility: {}\n", VR_GetVRInitErrorAsEnglishDescription(init_error));
		return EXIT_FAILURE;
	}

    vr::IVRApplications* apps = vr::VRApplications();
    vr::EVRApplicationError app_error = vr::VRApplicationError_None;

    bool currently_installed = apps->IsApplicationInstalled(application_key);
    std::string manifest_path = Path_MakeAbsolute(rel_manifest_path, Path_StripFilename(Path_GetExecutablePath()));
    if (install) {
        if (currently_installed) {
            if(apps->GetApplicationAutoLaunch(application_key) == false){
                fmt::print("Enabling auto-start.\n");
                apps->SetApplicationAutoLaunch(application_key, true);
            }else{
                fmt::print("Manifest is already installed and auto-start is enabled.\n");
            }

            return 0;
        }

        fmt::print("Attempting to install manifest.\n");
        
        app_error = apps->AddApplicationManifest(manifest_path.c_str());
        if (app_error != vr::VRApplicationError_None) {
            fmt::print("Could not install manifest: {}\n", apps->GetApplicationsErrorNameFromEnum(app_error));
            return EXIT_FAILURE;
        }

        app_error = apps->SetApplicationAutoLaunch(application_key, true);
        if (app_error != vr::VRApplicationError_None) {
            fmt::print("Could not set auto start: {}\n", apps->GetApplicationsErrorNameFromEnum(app_error));
            return EXIT_FAILURE;
        }
    } else if (currently_installed) { // disable
        if (apps->GetApplicationAutoLaunch(application_key) == true){
            fmt::print("Disabling auto-start\n");
            apps->SetApplicationAutoLaunch(application_key, false);
        }else{
            fmt::print("Auto-start is already disabled\n");
        }

        return 0;
    } else {
        fmt::print("Manifest is not installed.\n");
        return 0;
    }

    return 0;
}