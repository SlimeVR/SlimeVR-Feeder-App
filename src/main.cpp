#include <openvr.h>
#include <cstdio>
#include <csignal>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include <optional>
#include <cerrno>
#include <memory>

#include "pathtools_excerpt.h"
#include "matrix_utils.h"

using namespace vr;

// TODO: Temp Path
static const char* actions_path = "../../../bindings/actions.json";
static const char* pipe_name = "\\\\.\\pipe\\SlimeVRInput";

// Consider Standing universe
static const ETrackingUniverseOrigin tracking_origin = ETrackingUniverseOrigin::TrackingUniverseRawAndUncalibrated;

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

struct OpenVRStuff {
	IVRSystem* system = nullptr;
	IVRInput* input = nullptr;
	
	FILE* trackers_pipe = nullptr;

	VRActiveActionSet_t actionSet = { 0 };

	VRInputValueHandle_t value_handles[BodyPosition::BodyPosition_Count] = { k_ulInvalidInputValueHandle };

	VRActionHandle_t GetAction(const char* action_path) {
		VRActionHandle_t handle = k_ulInvalidInputValueHandle;
		EVRInputError error = (EVRInputError)input->GetActionHandle(action_path, &handle);
		if (error != VRInputError_None) {
			printf("Error: Unable to get action handle '%s': %d", action_path, error);
			// consider exiting?
		}

		return handle;
	}

	std::optional<std::vector<char>> GetStringProp(TrackedDeviceIndex_t index, ETrackedDeviceProperty prop) {
		vr::ETrackedPropertyError prop_error = ETrackedPropertyError::TrackedProp_Success;
		uint32_t size = system->GetStringTrackedDeviceProperty(index, prop, nullptr, 0, &prop_error);

		if (size == 0 || (prop_error != TrackedProp_Success && prop_error != TrackedProp_BufferTooSmall)) {
			if (prop_error != TrackedProp_Success) {
				printf("Error: size getting IVRSystem::GetStringTrackedDeviceProperty(%d): %d\n", prop, prop_error);
			}

			return std::nullopt;
		}

		std::vector<char> vec = std::vector<char>(size);
		system->GetStringTrackedDeviceProperty(index, prop, vec.data(), size, &prop_error);

		if (prop_error != TrackedProp_Success) {
			printf("Error: data getting IVRSystem::GetStringTrackedDeviceProperty(%d): %d\n", prop, prop_error);
			return std::nullopt;
		}

		return std::make_optional(vec);
	}

	std::optional<TrackedDeviceIndex_t> GetIndex(VRInputValueHandle_t value_handle) {
		InputOriginInfo_t info;
		EVRInputError error = input->GetOriginTrackedDeviceInfo(value_handle, &info, sizeof(info));
		if (error != EVRInputError::VRInputError_None) {
			printf("Error: IVRInput::GetOriginTrackedDeviceInfo: %d\n", error);
			return std::nullopt;
		}

		return std::make_optional(info.trackedDeviceIndex);
	}

	void Tick() {
		TrackedDevicePose_t device_poses[k_unMaxTrackedDeviceCount];

		system->GetDeviceToAbsoluteTrackingPose(tracking_origin, 0, device_poses, k_unMaxTrackedDeviceCount);
		// 0 is the hmd
		for (unsigned int jjj = 0; jjj < BodyPosition::BodyPosition_Count; ++jjj) {
			//printf("%s\n", actions[jjj]);
			VRInputValueHandle_t activeOrigin = value_handles[jjj];
			if (activeOrigin == k_ulInvalidInputValueHandle) {
				continue;
			}

			// TODO: should we get the index every tick, or should we just do that outside?
			std::optional<TrackedDeviceIndex_t> trackedDeviceIndex = GetIndex(activeOrigin);
			if (!trackedDeviceIndex.has_value()) {
				// already printed a message about this in GetIndex, just continue.
				continue;
			}

			TrackedDevicePose_t &pose = device_poses[trackedDeviceIndex.value()];

			// TODO: Filter out SlimeVR Generated trackers, if present.

			if (jjj != BodyPosition::Head) {
				// printf("skipping index %d\n", jjj);
				continue;
			} 

			HmdQuaternion_t rot = GetRotation(pose.mDeviceToAbsoluteTracking);
			HmdVector3_t pos = GetPosition(pose.mDeviceToAbsoluteTracking);

			// ensure we're using the same format as the driver for now.
			std::string s = std::to_string(pos.v[0]) +
				" " + std::to_string(pos.v[1]) +
				" " + std::to_string(pos.v[2]) +
				" " + std::to_string(rot.w) +
				" " + std::to_string(rot.x) +
				" " + std::to_string(rot.y) +
				" " + std::to_string(rot.z);

			// TODO: binary protocol?
			if (trackers_pipe) {
				//fprintf(trackers_pipe, "UPD %d %f %f %f %f %f %f %f\n", info.trackedDeviceIndex, pos.v[0], pos.v[1], pos.v[2], rot.w, rot.x, rot.y, rot.z);
				fprintf(trackers_pipe, "UPD %d %s\n", trackedDeviceIndex.value(), s.c_str());
			}
			//printf("UPD %d %f %f %f %f %f %f %f\n", info.trackedDeviceIndex, pos.v[0], pos.v[1], pos.v[2], rot.w, rot.x, rot.y, rot.z);
			printf("UPD %d %s\n", trackedDeviceIndex.value(), s.c_str());
		}
	}
};

volatile static sig_atomic_t should_exit = 0;

void handle_signal(int num) {
	// reinstall, in case it goes back to default.
	//signal(num, handle_signal);
	switch (num) {
	case SIGINT:
		should_exit = 1;
		break;
	}
}

void shutdown_vr(IVRSystem* _system) {
	VR_Shutdown();
}

int main(int argc, char* argv[]) {
	EVRInitError init_error = VRInitError_None;
	EVRInputError input_error = VRInputError_None;

	signal(SIGINT, handle_signal);

	OpenVRStuff stuff;

	// note: still windows only, i just wanted to be able to fprintf
	std::unique_ptr<FILE, decltype(&fclose)> trackers_pipe(fopen(pipe_name, "w"), &fclose);
	if (!trackers_pipe) {
		printf("Error opening pipe: '%s'\nContinuing...\n", strerror(errno));
		//return EXIT_FAILURE;
	}
	else {
		setvbuf(trackers_pipe.get(), nullptr, _IONBF, 0);
		stuff.trackers_pipe = trackers_pipe.get();
	}

	// maybe prefer Background?
	std::unique_ptr<IVRSystem, decltype(&shutdown_vr)> system(VR_Init(&init_error, VRApplication_Overlay), &shutdown_vr);
	if (init_error != VRInitError_None) {
		system = nullptr;
		printf("Unable to init VR runtime: %s\n", VR_GetVRInitErrorAsEnglishDescription(init_error));
		return EXIT_FAILURE;
	}

	stuff.system = system.get();
	stuff.input = VRInput();

	// Ensure VR Compositor is available, otherwise getting poses causes a crash (openvr v1.3.22)
	if (!VRCompositor()) {
		printf("Failed to initialize VR compositor!\n");
		return EXIT_FAILURE;
	}

	{
		std::string actionsFileName = Path_MakeAbsolute(actions_path, Path_StripFilename(Path_GetExecutablePath()));

		if ((input_error = stuff.input->SetActionManifestPath(actionsFileName.c_str())) != EVRInputError::VRInputError_None) {
			printf("Error: IVRInput::SetActionManifectPath: %d", input_error);
			return EXIT_FAILURE;
		}
	}

	VRActionHandle_t action_handles[BodyPosition::BodyPosition_Count];
	for (unsigned int iii = 0; iii < BodyPosition::BodyPosition_Count; ++iii) {
		action_handles[iii] = stuff.GetAction(actions[iii]);
	}

	VRActionSetHandle_t action_set_handle;
	if ((input_error = stuff.input->GetActionSetHandle("/actions/main", &action_set_handle)) != EVRInputError::VRInputError_None) {
		printf("Error: VRInput::GetActionSetHandle: %d", input_error);
		return EXIT_FAILURE;
	}

	stuff.actionSet = {
		action_set_handle,
		k_ulInvalidInputValueHandle,
		k_ulInvalidActionSetHandle,
		0,
		0
	};

	do {
		// wait for event
		while (true) {
			VREvent_t event;
			if (!system->PollNextEvent(&event, sizeof(event))) {
				std::this_thread::yield();
				continue;
			}

			// TODO: re-poll device indexes when events say we should?

			if (event.eventType == VREvent_Quit) {
				return 0;
			}

			if (event.eventType == 13337) {
				break; // got our vendor event, let's send some data.
			}
		}

		// TODO: do this in respose to controller appearing event or something -- we aren't using the position data from this.
		input_error = stuff.input->UpdateActionState(&stuff.actionSet, sizeof(stuff.actionSet), 1);
		if (input_error != EVRInputError::VRInputError_None) {
			printf("Error: IVRInput::UpdateActionState: %d\n", input_error);
		}
		else {
			for (unsigned int jjj = 0; jjj < BodyPosition::BodyPosition_Count; ++jjj) {
				InputPoseActionData_t pose;
				input_error = stuff.input->GetPoseActionDataRelativeToNow(action_handles[jjj], tracking_origin, 0, &pose, sizeof(pose), 0);
				if (input_error != EVRInputError::VRInputError_None) {
					printf("Error: IVRInput::GetPoseActionDataRelativeToNow: %d\n", input_error);
					continue;
				}

				// TODO: this could probably still use some refactoring.
				if (pose.bActive) {
					if (stuff.value_handles[jjj] != pose.activeOrigin) {
						stuff.value_handles[jjj] = pose.activeOrigin;
						std::optional<TrackedDeviceIndex_t> trackedDeviceIndex = stuff.GetIndex(pose.activeOrigin);
						if (!trackedDeviceIndex.has_value()) {
							// already printed a message about this in GetIndex, just continue.
							continue;
						}

						std::optional<std::vector<char>> controller_type = stuff.GetStringProp(trackedDeviceIndex.value(), ETrackedDeviceProperty::Prop_ControllerType_String);
						if (!controller_type.has_value()) {
							// uhhhhhhhhhhhhhhh
							if (stuff.trackers_pipe) {
								fprintf(stuff.trackers_pipe, "ADD %d %d Index%d\n", trackedDeviceIndex.value(), jjj, trackedDeviceIndex.value());
							}
							printf("ADD %d %d Index%d\n", trackedDeviceIndex.value(), jjj, trackedDeviceIndex.value());
						}
						else {
							if (stuff.trackers_pipe) {
								fprintf(stuff.trackers_pipe, "ADD %d %d %s\n", trackedDeviceIndex.value(), jjj, controller_type.value().data());
							}
							printf("ADD %d %d %s\n", trackedDeviceIndex.value(), jjj, controller_type.value().data());
						}
					}
				}
			}
		}

		stuff.Tick();
	} while (!should_exit);

	return 0;
}