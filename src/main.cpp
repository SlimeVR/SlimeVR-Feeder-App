#include <openvr.h>
#include <csignal>
#include <string>
#include <thread>
#include <chrono>
#include <vector>
#include <optional>
#include <cerrno>
#include <memory>
#include <fmt/core.h>
#include <fmt/ostream.h>
#include <iostream>
#include <fstream>

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
	// note: still windows only, i just wanted to be able to format
	std::ofstream trackers_pipe = std::ofstream(pipe_name, std::ios_base::binary | std::ios_base::ate);

	IVRSystem* system = nullptr;
	IVRInput* input = nullptr;

	VRActiveActionSet_t actionSet = { 0 };

	VRInputValueHandle_t value_handles[BodyPosition::BodyPosition_Count] = { k_ulInvalidInputValueHandle };

	VRActionHandle_t GetAction(const char* action_path) {
		VRActionHandle_t handle = k_ulInvalidInputValueHandle;
		EVRInputError error = (EVRInputError)input->GetActionHandle(action_path, &handle);
		if (error != VRInputError_None) {
			fmt::print("Error: Unable to get action handle '{}': {}", action_path, error);
			// consider exiting?
		}

		return handle;
	}

	std::optional<std::vector<char>> GetStringProp(TrackedDeviceIndex_t index, ETrackedDeviceProperty prop) {
		vr::ETrackedPropertyError prop_error = ETrackedPropertyError::TrackedProp_Success;
		uint32_t size = system->GetStringTrackedDeviceProperty(index, prop, nullptr, 0, &prop_error);

		if (size == 0 || (prop_error != TrackedProp_Success && prop_error != TrackedProp_BufferTooSmall)) {
			if (prop_error != TrackedProp_Success) {
				fmt::print("Error: size getting IVRSystem::GetStringTrackedDeviceProperty({}): {}\n", prop, prop_error);
			}

			return std::nullopt;
		}

		std::vector<char> vec = std::vector<char>(size);
		system->GetStringTrackedDeviceProperty(index, prop, vec.data(), size, &prop_error);

		if (prop_error != TrackedProp_Success) {
			fmt::print("Error: data getting IVRSystem::GetStringTrackedDeviceProperty({}): {}\n", prop, prop_error);
			return std::nullopt;
		}

		return std::make_optional(vec);
	}

	std::optional<TrackedDeviceIndex_t> GetIndex(VRInputValueHandle_t value_handle) {
		InputOriginInfo_t info;
		EVRInputError error = input->GetOriginTrackedDeviceInfo(value_handle, &info, sizeof(info));
		if (error != EVRInputError::VRInputError_None) {
			fmt::print("Error: IVRInput::GetOriginTrackedDeviceInfo: {}\n", error);
			return std::nullopt;
		}

		return std::make_optional(info.trackedDeviceIndex);
	}

	void Tick() {
		TrackedDevicePose_t device_poses[k_unMaxTrackedDeviceCount];

		system->GetDeviceToAbsoluteTrackingPose(tracking_origin, 0, device_poses, k_unMaxTrackedDeviceCount);
		for (unsigned int jjj = 0; jjj < BodyPosition::BodyPosition_Count; ++jjj) {
			VRInputValueHandle_t activeOrigin = value_handles[jjj];
			if (activeOrigin == k_ulInvalidInputValueHandle) {
				continue;
			}

			// TODO: profile this. should we get the index every tick, or should we just do that outside?
			std::optional<TrackedDeviceIndex_t> trackedDeviceIndex = GetIndex(activeOrigin);
			if (!trackedDeviceIndex.has_value()) {
				// already printed a message about this in GetIndex, just continue.
				continue;
			}

			TrackedDevicePose_t &pose = device_poses[trackedDeviceIndex.value()];

			// TODO: Filter out SlimeVR Generated trackers, if present.

			HmdQuaternion_t rot = GetRotation(pose.mDeviceToAbsoluteTracking);
			HmdVector3_t pos = GetPosition(pose.mDeviceToAbsoluteTracking);

			// TODO: no point in running the tick if we can't write?
			if (trackers_pipe.is_open()) {
				fmt::print(trackers_pipe, "UPD {:d} {:f} {:f} {:f} {:f} {:f} {:f} {:f}\n", trackedDeviceIndex.value(), pos.v[0], pos.v[1], pos.v[2], rot.w, rot.x, rot.y, rot.z);
				trackers_pipe.flush();
			}
		}
	}

	
	void UpdateValueHandles(VRActionHandle_t(&actions)[BodyPosition::BodyPosition_Count]) {
		EVRInputError input_error = input->UpdateActionState(&actionSet, sizeof(actionSet), 1);
		if (input_error != EVRInputError::VRInputError_None) {
			fmt::print("Error: IVRInput::UpdateActionState: {}\n", input_error);
			return;
		}

		for (unsigned int jjj = 0; jjj < BodyPosition::BodyPosition_Count; ++jjj) {
			InputPoseActionData_t pose;
			input_error = input->GetPoseActionDataRelativeToNow(actions[jjj], tracking_origin, 0, &pose, sizeof(pose), 0);
			if (input_error != EVRInputError::VRInputError_None) {
				fmt::print("Error: IVRInput::GetPoseActionDataRelativeToNow: {}\n", input_error);
				continue;
			}

			// TODO: this could maybe still use some refactoring.
			// TODO: report STAtus
			if (pose.bActive) {
				if (value_handles[jjj] != pose.activeOrigin) {
					value_handles[jjj] = pose.activeOrigin;
					std::optional<TrackedDeviceIndex_t> trackedDeviceIndex = GetIndex(pose.activeOrigin);
					if (!trackedDeviceIndex.has_value()) {
						// already printed a message about this in GetIndex, just continue.
						continue;
					}

					std::optional<std::vector<char>> controller_type = GetStringProp(trackedDeviceIndex.value(), ETrackedDeviceProperty::Prop_ControllerType_String);
					if (!controller_type.has_value()) {
						// uhhhhhhhhhhhhhhh
						if (trackers_pipe.is_open()) {
							fmt::print(trackers_pipe, "ADD {} {} Index{:d}\n", trackedDeviceIndex.value(), jjj, trackedDeviceIndex.value());
						}
					}
					else {
						if (trackers_pipe.is_open()) {
							fmt::print(trackers_pipe, "ADD {} {} {}\n", trackedDeviceIndex.value(), jjj, controller_type.value().data());
						}
					}
				}
			}
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

	if (stuff.trackers_pipe.fail()) {
		std::cout << "Error opening pipe. Continuing..." << std::endl;
		// TODO: attempt to re-open pipe periodically, to support server coming up after application?
	}

	// maybe prefer Background?
	std::unique_ptr<IVRSystem, decltype(&shutdown_vr)> system(VR_Init(&init_error, VRApplication_Overlay), &shutdown_vr);
	if (init_error != VRInitError_None) {
		system = nullptr;
		fmt::print("Unable to init VR runtime: {}\n", VR_GetVRInitErrorAsEnglishDescription(init_error));
		return EXIT_FAILURE;
	}

	stuff.system = system.get();
	stuff.input = VRInput();

	// Ensure VR Compositor is available, otherwise getting poses causes a crash (openvr v1.3.22)
	if (!VRCompositor()) {
		std::cout << "Failed to initialize VR compositor!" << std::endl;
		return EXIT_FAILURE;
	}

	{
		std::string actionsFileName = Path_MakeAbsolute(actions_path, Path_StripFilename(Path_GetExecutablePath()));

		if ((input_error = stuff.input->SetActionManifestPath(actionsFileName.c_str())) != EVRInputError::VRInputError_None) {
			fmt::print("Error: IVRInput::SetActionManifectPath: {}\n", input_error);
			return EXIT_FAILURE;
		}
	}

	VRActionHandle_t action_handles[BodyPosition::BodyPosition_Count];
	for (unsigned int iii = 0; iii < BodyPosition::BodyPosition_Count; ++iii) {
		action_handles[iii] = stuff.GetAction(actions[iii]);
	}

	VRActionSetHandle_t action_set_handle;
	if ((input_error = stuff.input->GetActionSetHandle("/actions/main", &action_set_handle)) != EVRInputError::VRInputError_None) {
		fmt::print("Error: VRInput::GetActionSetHandle: {}\n", input_error);
		return EXIT_FAILURE;
	}

	stuff.actionSet = {
		action_set_handle,
		k_ulInvalidInputValueHandle,
		k_ulInvalidActionSetHandle,
		0,
		0
	};

	stuff.UpdateValueHandles(action_handles);

	// event loop
	while (!should_exit) {
		VREvent_t event;
		if (system->PollNextEvent(&event, sizeof(event))) {
			switch (event.eventType) {
			case VREvent_Quit:
				return 0;

			// TODO: add more events, or remove some events?
			case VREvent_TrackedDeviceActivated:
			case VREvent_TrackedDeviceDeactivated:
			case VREvent_TrackedDeviceRoleChanged:
			case VREvent_TrackedDeviceUpdated:
			case VREvent_DashboardDeactivated:
				//stuff.UpdateValueHandles(action_handles);
				break;

			default:
				fmt::print("Unhandled event: {}({})\n", stuff.system->GetEventTypeNameFromEnum((EVREventType)event.eventType), event.eventType);
			}
		}

		// TODO: don't do this every loop, we really shouldn't need to.
		stuff.UpdateValueHandles(action_handles);

		stuff.Tick();

		// run as fast as possible (for now), but make sure that we give time back to the OS.
		std::this_thread::yield();
	}

	return 0;
}