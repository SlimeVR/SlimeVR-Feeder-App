#include <openvr.h>
#include <cstdio>
#include <string>
#include "pathtools_excerpt.h"

using namespace vr;

// TODO: TEMP
static const char* actions_path = "../../../bindings/actions.json";

enum BodyPosition {
	Head = 0,
	LeftHand,
	RightHand,
	LeftFoot,
	RightFoot,
	LeftShoulder,
	RightShoulder,
	LeftElbow,
	RightElbow,
	LeftKnee,
	RightKnee,
	Waist,
	Chest,
	BodyPosition_Count
};

struct Role {
	enum BodyPosition position;
	const char* input_source_path;
};

static const char* actions[BodyPosition::BodyPosition_Count] = {
	"/actions/main/in/head",
	"/actions/main/in/left_hand",
	"/actions/main/in/right_hand",
	"/actions/main/in/left_foot",
	"/actions/main/in/right_foot",
	"/actions/main/in/left_shoulder",
	"/actions/main/in/right_shoulder",
	"/actions/main/in/left_elbow",
	"/actions/main/in/right_elbow",
	"/actions/main/in/left_knee",
	"/actions/main/in/right_knee",
	"/actions/main/in/waist",
	"/actions/main/in/chest"
};

VRActionHandle_t get_action(const char* action_path) {
	VRActionHandle_t handle = k_ulInvalidInputValueHandle;
	EVRInputError error = VRInput()->GetActionHandle(action_path, &handle);
	if (error != VRInputError_None) {
		printf("Error: Unable to get action handle '%s': %d", action_path, error);
		// consider exiting?
	}

	return handle;
}

int main(int argc, char* argv[]) {
	int ret = 0;
	EVRInitError error = VRInitError_None;
	// maybe prefer Background?

	IVRSystem *system = VR_Init(&error, VRApplication_Overlay);
	if (error != VRInitError_None) {
		system = nullptr;
		printf("Unable to init VR runtime: %s\n", VR_GetVRInitErrorAsEnglishDescription(error));
		ret = EXIT_FAILURE;
		goto ret;
	}

	// Ensure VR Compositor is available, otherwise getting poses causes a crash (openvr v1.3.22)
	if (!VRCompositor()) {
		printf("Failed to initialize VR compositor!\n");
		ret = EXIT_FAILURE;
		goto shutdown;
	}

	{
		std::string actionsFileName = Path_MakeAbsolute(actions_path, Path_StripFilename(Path_GetExecutablePath()));

		EVRInputError input_error;
		if ((input_error = VRInput()->SetActionManifestPath(actionsFileName.c_str())) != EVRInputError::VRInputError_None) {
			printf("Error: IVRInput::SetActionManifectPath: %d", input_error);
			ret = EXIT_FAILURE;
			goto shutdown;
		}
	}

	VRActionHandle_t handles[BodyPosition::BodyPosition_Count];
	for (unsigned int iii = 0; iii < BodyPosition::BodyPosition_Count; ++iii) {
		handles[iii] = get_action(actions[iii]);
	}

	VRActionSetHandle_t action_set_handle;
	{
		EVRInputError error = VRInput()->GetActionSetHandle("/actions/main", &action_set_handle);
		if (error != EVRInputError::VRInputError_None) {
			printf("Error: VRInput::GetActionSetHandle: %d", error);
			ret = EXIT_FAILURE;
			goto shutdown;
		}
	}

	VRActiveActionSet_t actionSet = {
		action_set_handle,
		k_ulInvalidInputValueHandle,
		k_ulInvalidActionSetHandle,
		0,
		0
	};

	char controller_type[k_unMaxPropertyStringSize];
	do {
		EVRInputError error = VRInput()->UpdateActionState(&actionSet, sizeof(actionSet), 1);
		if (error != EVRInputError::VRInputError_None) {
			printf("Error: IVRInput::UpdateActionState: %d\n", error);
		}
		// 0 is the hmd
		for (unsigned int jjj = 0; jjj < BodyPosition::BodyPosition_Count; ++jjj) {
			printf("%s\n", actions[jjj]);
			InputPoseActionData_t pose;
			// Consider Standing universe
			error = VRInput()->GetPoseActionDataRelativeToNow(handles[jjj], ETrackingUniverseOrigin::TrackingUniverseRawAndUncalibrated, 0, &pose, sizeof(pose), k_ulInvalidInputValueHandle);
			if (error != EVRInputError::VRInputError_None) {
				printf("Error: IVRInput::GetPoseActionDataRelativeToNow: %d\n", error);
				continue;
			}
			if (!pose.bActive) {
				printf("inactive\n");
				continue;
			}

			InputOriginInfo_t info;
			error = VRInput()->GetOriginTrackedDeviceInfo(pose.activeOrigin, &info, sizeof(info));
			if (error != EVRInputError::VRInputError_None) {
				printf("Error: IVRInput::GetOriginTrackedDeviceInfo: %d\n", error);
				continue;
			}

			ETrackedPropertyError prop_error;
			system->GetStringTrackedDeviceProperty(info.trackedDeviceIndex, Prop_ControllerType_String, controller_type, k_unMaxPropertyStringSize, &prop_error);
			if (error != TrackedProp_Success) {
				printf("Error: IVRSystem::GetStringTrackedDeviceProperty(%d, Prop_ControllerType_String): %d\n", jjj, prop_error);
				continue;
			}
			printf("%s\n", controller_type);
		}
	} while (false);

	shutdown:
	VR_Shutdown();
	ret:
	return ret;
}