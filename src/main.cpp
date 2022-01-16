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
#include "bridge.hpp"
#include <ProtobufMessages.pb.h>

using namespace vr;

// TODO: Temp Path
static constexpr char* actions_path = "./bindings/actions.json";
static constexpr char* pipe_name = "\\\\.\\pipe\\SlimeVRInput";

// Consider Standing universe
//static constexpr ETrackingUniverseOrigin tracking_origin = ETrackingUniverseOrigin::TrackingUniverseRawAndUncalibrated;
static constexpr ETrackingUniverseOrigin tracking_origin = ETrackingUniverseOrigin::TrackingUniverseStanding;

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

static constexpr char* positionNames[BodyPosition::BodyPosition_Count] = {
	"Head",
	"LeftHand",
	"RightHand",
	"LeftFoot",
	"RightFoot",
	"LeftShoulder",
	"RightShoulder",
	"LeftElbow",
	"RightElbow",
	"LeftKnee",
	"RightKnee",
	"Waist",
	"Chest"
};

static constexpr char* actions[BodyPosition::BodyPosition_Count] = {
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

class Tracker {
	SlimeVRBridge &bridge;
	HmdQuaternion_t current_rotation = { 0 };
	HmdVector3_t current_position = { 0 };
	TrackedDeviceIndex_t index = k_unTrackedDeviceIndexInvalid;

	bool current_valid = false;
public:
	Tracker(SlimeVRBridge &bridge): bridge(bridge) {}

	void SendStatus(messages::TrackerStatus_Status status_val) {
		messages::ProtobufMessage message;
		messages::TrackerStatus *status = message.mutable_tracker_status();

		status->set_status(status_val);
		status->set_tracker_id(index);

		bridge.sendMessage(message);

		fmt::print("Device (Index {}) status: {} ({})\n", index, messages::TrackerStatus_Status_Name(status_val), status_val);
	}

	void Update(TrackedDevicePose_t pose, bool just_connected) {
		if (pose.bPoseIsValid) {
			if (!current_valid) {
				current_valid = true;
				SendStatus(messages::TrackerStatus_Status_OK);
			}

			HmdQuaternion_t new_rotation = GetRotation(pose.mDeviceToAbsoluteTracking);
			HmdVector3_t new_position = GetPosition(pose.mDeviceToAbsoluteTracking);

			// only update if there's a difference, or if we just connected
			if (
				just_connected ||
				current_rotation.w != new_rotation.w || current_rotation.x != new_rotation.x ||
				current_rotation.y != new_rotation.y || current_rotation.z != new_rotation.z ||
				current_position.v[0] != new_position.v[0] || current_position.v[1] != new_position.v[1] ||
				current_position.v[2] != new_position.v[2]
			) {
				current_position = new_position;
				current_rotation = new_rotation;

				// send our position message
				messages::ProtobufMessage message;
				messages::Position *position = message.mutable_position();
				position->set_x(current_position.v[0]);
				position->set_y(current_position.v[1]);
				position->set_z(current_position.v[2]);
				position->set_qw(current_rotation.w);
				position->set_qx(current_rotation.x);
				position->set_qy(current_rotation.y);
				position->set_qz(current_rotation.z);
				position->set_tracker_id(index);
				position->set_data_source(
					pose.eTrackingResult == ETrackingResult::TrackingResult_Fallback_RotationOnly
					? messages::Position_DataSource_IMU
					: messages::Position_DataSource_FULL
				);

				bridge.sendMessage(message);
			}
		}
		
		// send status update on change, or if we just connected.
		if (current_valid && !pose.bPoseIsValid || just_connected) {
			current_valid = false;
			current_position = {0};
			current_rotation = {0};
			if (!pose.bDeviceIsConnected) {
				SendStatus(messages::TrackerStatus_Status_DISCONNECTED);
			} else if (pose.eTrackingResult == ETrackingResult::TrackingResult_Running_OutOfRange) {
				SendStatus(messages::TrackerStatus_Status_OCCLUDED);
			} else {
				SendStatus(messages::TrackerStatus_Status_ERROR);
			}
		}
	}

	template <typename N, typename S>
	void SetIndex(TrackedDeviceIndex_t idx, BodyPosition pos, N &&get_name, S &&get_serial, bool send_anyway) {
		if (index != k_unTrackedDeviceIndexInvalid) {
			fmt::print("Warning: Tracked Device Index changed from {} to {}. Report this, because assumptions were incorrectly made.", index, idx);
		}

		if (index != idx || send_anyway) {
			index = idx;

			auto name = get_name();
			std::optional<std::string> serial = get_serial();

			messages::ProtobufMessage message;
			messages::TrackerAdded *added = message.mutable_tracker_added();
			added->set_tracker_id(index);
			added->set_tracker_role(pos);
			added->set_tracker_name(name);
			if (serial.has_value()) {
				added->set_tracker_serial(serial.value());
			}

			bridge.sendMessage(message);

			// log it.
			fmt::print("Found device \"{}\" at {} with index {}\n", name, positionNames[pos], index);
		}
	}
};

struct OpenVRStuff {
	// note: still windows only, i just wanted to be able to format
	SlimeVRBridge &bridge;

	OpenVRStuff(SlimeVRBridge &bridge): bridge(bridge), trackers {
		// UGH
		Tracker(bridge), Tracker(bridge), Tracker(bridge), Tracker(bridge), Tracker(bridge), Tracker(bridge), Tracker(bridge),
		Tracker(bridge), Tracker(bridge), Tracker(bridge), Tracker(bridge), Tracker(bridge), Tracker(bridge)
	} {}

	IVRSystem* system = nullptr;
	IVRInput* input = nullptr;

	VRActiveActionSet_t actionSet = { 0 };

	VRInputValueHandle_t value_handles[BodyPosition::BodyPosition_Count] = { k_ulInvalidInputValueHandle };
	Tracker trackers[BodyPosition::BodyPosition_Count];

	VRActionHandle_t GetAction(const char* action_path) {
		VRActionHandle_t handle = k_ulInvalidInputValueHandle;
		EVRInputError error = (EVRInputError)input->GetActionHandle(action_path, &handle);
		if (error != VRInputError_None) {
			fmt::print("Error: Unable to get action handle '{}': {}", action_path, error);
			// consider exiting?
		}

		return handle;
	}

	std::optional<std::string> GetStringProp(TrackedDeviceIndex_t index, ETrackedDeviceProperty prop) {
		vr::ETrackedPropertyError prop_error = ETrackedPropertyError::TrackedProp_Success;
		uint32_t size = system->GetStringTrackedDeviceProperty(index, prop, nullptr, 0, &prop_error);

		if (size == 0 || (prop_error != TrackedProp_Success && prop_error != TrackedProp_BufferTooSmall)) {
			if (prop_error != TrackedProp_Success) {
				fmt::print("Error: size getting IVRSystem::GetStringTrackedDeviceProperty({}): {}\n", prop, prop_error);
			}

			return std::nullopt;
		}

		std::string prop_value = std::string(size, '\0');
		system->GetStringTrackedDeviceProperty(index, prop, prop_value.data(), size, &prop_error);
		prop_value.resize(size-1);

		if (prop_error != TrackedProp_Success) {
			fmt::print("Error: data getting IVRSystem::GetStringTrackedDeviceProperty({}): {}\n", prop, prop_error);
			return std::nullopt;
		}

		return std::make_optional(prop_value);
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

	void Tick(bool just_connected) {
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
				// this handle is no longer valid, discard it.
				value_handles[jjj] = k_ulInvalidInputValueHandle;
				// TODO: notify the server somehow.
				// TODO: reset tracker object.
				// already printed a message about this in GetIndex, so don't need additional logging.
				continue;
			}

			// TODO: Filter out SlimeVR Generated trackers, if applicable.

			TrackedDevicePose_t &pose = device_poses[trackedDeviceIndex.value()];

			trackers[jjj].Update(pose, just_connected);
		}
	}

	void UpdateValueHandles(VRActionHandle_t(&actions)[BodyPosition::BodyPosition_Count], bool just_connected) {
		EVRInputError input_error = input->UpdateActionState(&actionSet, sizeof(VRActiveActionSet_t), 1);
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

			if (pose.bActive) {
				if (value_handles[jjj] != pose.activeOrigin) {
					value_handles[jjj] = pose.activeOrigin;
					std::optional<TrackedDeviceIndex_t> trackedDeviceIndex = GetIndex(pose.activeOrigin);
					if (!trackedDeviceIndex.has_value()) {
						// already printed a message about this in GetIndex, just continue.
						continue;
					}

					auto get_name = [&, this]() {
						auto controller_type = this->GetStringProp(trackedDeviceIndex.value(), ETrackedDeviceProperty::Prop_ControllerType_String);
						
						if (controller_type.has_value()) {
							return controller_type.value();
						} else {
							// uhhhhhhhhhhhhhhh
							return fmt::format("Index{}", trackedDeviceIndex.value());
						}
					};

					auto get_serial = [&, this]() {
						return this->GetStringProp(trackedDeviceIndex.value(), ETrackedDeviceProperty::Prop_SerialNumber_String);
					};

					trackers[jjj].SetIndex(trackedDeviceIndex.value(), (BodyPosition)jjj, get_name, get_serial, just_connected);
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
	GOOGLE_PROTOBUF_VERIFY_VERSION;

	EVRInitError init_error = VRInitError_None;
	EVRInputError input_error = VRInputError_None;

	signal(SIGINT, handle_signal);
	auto bridge = SlimeVRBridge::factory();
	OpenVRStuff stuff(*bridge);

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

	VRActionHandle_t calibration_action = stuff.GetAction("/actions/main/in/request_calibration");

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

	stuff.UpdateValueHandles(action_handles, false);

	// event loop
	while (!should_exit) {
		bool just_connected = bridge->runFrame();

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
				// stuff.UpdateValueHandles(action_handles, just_connected);
				break;

			default:
				// fmt::print("Unhandled event: {}({})\n", stuff.system->GetEventTypeNameFromEnum((EVREventType)event.eventType), event.eventType);
				// I'm not relying on events to actually trigger anything right now, so don't bother printing anything.
				break;
			}
		}

		messages::ProtobufMessage recievedMessage;
		// TODO: I don't think there are any messages from the server that we care about at the moment, but let's make sure to not let the pipe fill up.
		bridge->getNextMessage(recievedMessage);

		// TODO: don't do this every loop, we really shouldn't need to.
		stuff.UpdateValueHandles(action_handles, just_connected);

		InputDigitalActionData_t calibration_data;
		input_error = stuff.input->GetDigitalActionData(calibration_action, &calibration_data, sizeof(InputDigitalActionData_t), 0);
		if (input_error == EVRInputError::VRInputError_None) {
			constexpr bool falling_edge = false; // rising edge for now, making it easy to switch for now just in case.
			if (calibration_data.bChanged && (calibration_data.bState ^ falling_edge)) {
				fmt::print("User requested calibration!\n");
				messages::ProtobufMessage message;
				messages::UserAction *userAction = message.mutable_user_action();
				userAction->set_name("Request Calibration"); // TODO: what specific name should be used?
				// TODO: does the server actually care or is able to make use of this information?
				// should i encode it as a binary value encoded to string instead? (i.e. reinterpret float as int, then pass 0x{})
				userAction->mutable_action_arguments()->insert({ "fUpdateTime", fmt::format("{}", calibration_data.fUpdateTime) });

				bridge->sendMessage(message);
			}
		} else {
			fmt::print("Error: VRInput::GetDigitalActionData: {}\n", input_error);
		}
		

		stuff.Tick(just_connected);

		// run as fast as possible (for now), but make sure that we give time back to the OS.
		std::this_thread::yield();
	}

	return 0;
}