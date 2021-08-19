#include <openvr.h>
#include <cstdio>
#include <csignal>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include <optional>
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

std::optional<std::vector<char>> get_string_prop(IVRSystem *system, TrackedDeviceIndex_t index, ETrackedDeviceProperty prop) {
	ETrackedPropertyError error;
	uint32_t size = system->GetStringTrackedDeviceProperty(index, prop, nullptr, 0, &error);

	if (size == 0 || (error != TrackedProp_Success && error != TrackedProp_BufferTooSmall)) {
		if (error != TrackedProp_Success) {
			printf("Error: size getting IVRSystem::GetStringTrackedDeviceProperty(%d): %d\n", prop, error);
		}
		
		return std::nullopt;
	}

	std::vector<char> vec = std::vector<char>(size);
	system->GetStringTrackedDeviceProperty(index, prop, vec.data(), size, &error);

	if (error != TrackedProp_Success) {
		printf("Error: data getting IVRSystem::GetStringTrackedDeviceProperty(%d): %d\n", prop, error);
		return std::nullopt;
	}

	return std::make_optional(vec);
}

HmdQuaternion_t GetRotation(HmdMatrix34_t matrix) {
	vr::HmdQuaternion_t q;

	q.w = sqrt(fmax(0, 1 + matrix.m[0][0] + matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = sqrt(fmax(0, 1 + matrix.m[0][0] - matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.y = sqrt(fmax(0, 1 - matrix.m[0][0] + matrix.m[1][1] - matrix.m[2][2])) / 2;
	q.z = sqrt(fmax(0, 1 - matrix.m[0][0] - matrix.m[1][1] + matrix.m[2][2])) / 2;
	q.x = copysign(q.x, matrix.m[2][1] - matrix.m[1][2]);
	q.y = copysign(q.y, matrix.m[0][2] - matrix.m[2][0]);
	q.z = copysign(q.z, matrix.m[1][0] - matrix.m[0][1]);
	return q;
}
//-----------------------------------------------------------------------------
// Purpose: Extracts position (x,y,z).
// from: https://github.com/Omnifinity/OpenVR-Tracking-Example/blob/master/HTC%20Lighthouse%20Tracking%20Example/LighthouseTracking.cpp
//-----------------------------------------------------------------------------

HmdVector3_t GetPosition(HmdMatrix34_t matrix) {
	vr::HmdVector3_t vector;

	vector.v[0] = matrix.m[0][3];
	vector.v[1] = matrix.m[1][3];
	vector.v[2] = matrix.m[2][3];

	return vector;
}

volatile static sig_atomic_t got_sigint = 0;

void handle_signal(int num) {
	// reinstall, in case it goes back to default.
	//signal(num, handle_signal);
	switch (num) {
	case SIGINT:
		got_sigint = 1;
		break;
	}
}

int main(int argc, char* argv[]) {
	int ret = 0;
	EVRInitError error = VRInitError_None;
	EVRInputError input_error = VRInputError_None;

	signal(SIGINT, handle_signal);

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

		if ((input_error = VRInput()->SetActionManifestPath(actionsFileName.c_str())) != EVRInputError::VRInputError_None) {
			printf("Error: IVRInput::SetActionManifectPath: %d", input_error);
			ret = EXIT_FAILURE;
			goto shutdown;
		}
	}

	// note: still windows only, i just wanted to be able to fprintf
	//FILE* trackers_pipe = fopen("\\\\.\\pipe\\TrackersPipe", "w");

	VRActionHandle_t action_handles[BodyPosition::BodyPosition_Count];
	for (unsigned int iii = 0; iii < BodyPosition::BodyPosition_Count; ++iii) {
		action_handles[iii] = get_action(actions[iii]);
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

	VRInputValueHandle_t value_handles[BodyPosition::BodyPosition_Count] = { k_ulInvalidInputValueHandle };

	do {
		TrackedDevicePose_t device_poses[k_unMaxTrackedDeviceCount];

		std::this_thread::sleep_for(std::chrono::seconds(1));
		EVRInputError error = VRInput()->UpdateActionState(&actionSet, sizeof(actionSet), 1);
		VRSystem()->GetDeviceToAbsoluteTrackingPose(TrackingUniverseRawAndUncalibrated, 0, device_poses, k_unMaxTrackedDeviceCount);
		if (error != EVRInputError::VRInputError_None) {
			printf("Error: IVRInput::UpdateActionState: %d\n", error);
		}
		// 0 is the hmd
		for (unsigned int jjj = 0; jjj < BodyPosition::BodyPosition_Count; ++jjj) {
			//printf("%s\n", actions[jjj]);
			InputPoseActionData_t pose;
			// Consider Standing universe
			error = VRInput()->GetPoseActionDataRelativeToNow(action_handles[jjj], ETrackingUniverseOrigin::TrackingUniverseRawAndUncalibrated, 0, &pose, sizeof(pose), k_ulInvalidInputValueHandle);
			if (error != EVRInputError::VRInputError_None) {
				printf("Error: IVRInput::GetPoseActionDataRelativeToNow: %d\n", error);
				continue;
			}
			if (!pose.bActive) {
				pose.activeOrigin = value_handles[jjj];
			}
			else {
				value_handles[jjj] = pose.activeOrigin;
			}

			if (pose.activeOrigin == k_ulInvalidInputValueHandle) {
				continue;
			}

			// TODO: Filter out SlimeVR Generated trackers, if present.
			InputOriginInfo_t info;
			error = VRInput()->GetOriginTrackedDeviceInfo(pose.activeOrigin, &info, sizeof(info));
			if (error != EVRInputError::VRInputError_None) {
				printf("Error: IVRInput::GetOriginTrackedDeviceInfo: %d\n", error);
				continue;
			}

			if (!pose.bActive) {
				if (device_poses[info.trackedDeviceIndex].bPoseIsValid) {
					pose.pose = device_poses[info.trackedDeviceIndex];
				}
				else {
					printf("no valid pose found for '%s'\n", actions[jjj]);
					continue;
				}
			}

			// i suppose we have the option of allocating a static buffer for this, and avoiding allocation
			/*std::optional<std::vector<char>> controller_type = get_string_prop(system, info.trackedDeviceIndex, ETrackedDeviceProperty::Prop_ControllerType_String);
			if (!controller_type.has_value()) {
				continue;
			}

			printf("%s\n", controller_type.value().data());*/

			if (jjj != BodyPosition::Head) {
				// printf("skipping index %d\n", jjj);
				continue;
			}

			HmdQuaternion_t rot = GetRotation(pose.pose.mDeviceToAbsoluteTracking);
			HmdVector3_t pos = GetPosition(pose.pose.mDeviceToAbsoluteTracking);

			// TODO: binary protocol?
			printf("%f %f %f %f %f %f %f\n", pos.v[0], pos.v[1], pos.v[2], rot.w, rot.x, rot.y, rot.z);
		}
	} while (!got_sigint);
	printf("cleaning up\n");
	shutdown:
	VR_Shutdown();
	ret:
	return ret;
}