#include <openvr.h>
#include <cstdio>
#include <csignal>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include <optional>
#include "pathtools_excerpt.h"
#include <cerrno>

using namespace vr;

// TODO: TEMP
static const char* actions_path = "../../../bindings/actions.json";

static const char* pipe_name = "\\\\.\\pipe\\SlimeVRInput";

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
	int ret = EXIT_SUCCESS;
	EVRInitError error = VRInitError_None;
	EVRInputError input_error = VRInputError_None;

	signal(SIGINT, handle_signal);

	// note: still windows only, i just wanted to be able to fprintf
	FILE* trackers_pipe = fopen(pipe_name, "w");
	if (!trackers_pipe) {
		printf("%s\n", strerror(errno));
		ret = EXIT_FAILURE;
		goto ret;
	}

	setvbuf(trackers_pipe, nullptr, _IONBF, 0);

	// maybe prefer Background?
	IVRSystem *system = VR_Init(&error, VRApplication_Overlay);
	if (error != VRInitError_None) {
		system = nullptr;
		printf("Unable to init VR runtime: %s\n", VR_GetVRInitErrorAsEnglishDescription(error));
		ret = EXIT_FAILURE;
		goto close_pipe;
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
		// wait for event
		while (true) {
			VREvent_t event;
			if (!system->PollNextEvent(&event, sizeof(event))) {
				std::this_thread::yield();
				continue;
			}

			if (event.eventType == 13337) {
				break; // got our vendor event.
			}
		}

		TrackedDevicePose_t device_poses[k_unMaxTrackedDeviceCount];

		
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
			bool send_add = false;
			if (!pose.bActive) {
				pose.activeOrigin = value_handles[jjj];
			}
			else if (value_handles[jjj] != pose.activeOrigin) {
				send_add = true;
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
					//printf("no valid pose found for '%s'\n", actions[jjj]);
					continue;
				}
			}

			// i suppose we have the option of allocating a static buffer for this, and avoiding allocation
			std::optional<std::vector<char>> controller_type = get_string_prop(system, info.trackedDeviceIndex, ETrackedDeviceProperty::Prop_ControllerType_String);
			if (!controller_type.has_value()) {
				continue;
			}

			//printf("%s\n", controller_type.value().data());

			if (jjj != BodyPosition::Head) {
				// printf("skipping index %d\n", jjj);
				continue;
			}

			if (send_add) {
				fprintf(trackers_pipe, "ADD %d %d %s\n", info.trackedDeviceIndex, jjj, controller_type.value().data());
				printf("ADD %d %d %s\n", info.trackedDeviceIndex, jjj, controller_type.value().data());
			}

			HmdQuaternion_t rot = GetRotation(pose.pose.mDeviceToAbsoluteTracking);
			HmdVector3_t pos = GetPosition(pose.pose.mDeviceToAbsoluteTracking);

			// ensure we're using the same format as the driver for now.
			std::string s = std::to_string(pos.v[0]) +
				" " + std::to_string(pos.v[1]) +
				" " + std::to_string(pos.v[2]) +
				" " + std::to_string(rot.w) +
				" " + std::to_string(rot.x) +
				" " + std::to_string(rot.y) +
				" " + std::to_string(rot.z);

			// TODO: binary protocol?
			//fprintf(trackers_pipe, "UPD %d %f %f %f %f %f %f %f\n", info.trackedDeviceIndex, pos.v[0], pos.v[1], pos.v[2], rot.w, rot.x, rot.y, rot.z);
			//printf("UPD %d %f %f %f %f %f %f %f\n", info.trackedDeviceIndex, pos.v[0], pos.v[1], pos.v[2], rot.w, rot.x, rot.y, rot.z);
			fprintf(trackers_pipe, "UPD %d %s\n", info.trackedDeviceIndex, s.c_str());
			printf("UPD %d %s\n", info.trackedDeviceIndex, s.c_str());
		}
	} while (!got_sigint);
	printf("cleaning up\n");
shutdown:
	VR_Shutdown();
close_pipe:
	fclose(trackers_pipe);
ret:
	return ret;
}