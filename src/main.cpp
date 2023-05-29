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
#include <simdjson.h>
#include <args.hxx>

#include "pathtools_excerpt.h"
#include "matrix_utils.h"
#include "bridge.hpp"
#include "setup.hpp"
#include "version.h"
#include <ProtobufMessages.pb.h>

using namespace vr;

// TODO: Temp Path
static constexpr const char* actions_path = "./bindings/actions.json";
static constexpr const char* config_path = "./config.txt";

enum class BodyPosition {
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

// TODO: keep track of things as SlimeVRPosition in the first place.
enum class SlimeVRPosition {
	None = 0,
	Waist,
	LeftFoot,
	RightFoot,
	Chest,
	LeftKnee,
	RightKnee,
	LeftElbow,
	RightElbow,
	LeftShoulder,
	RightShoulder,
	LeftHand,
	RightHand,
	LeftController,
	RightController,
	Head,
	Neck,
	Camera,
	Keyboard,
	HMD,
	Beacon,
	GenericController
};

static constexpr SlimeVRPosition positionIDs[(int)BodyPosition::BodyPosition_Count] = {
	SlimeVRPosition::Head,
	SlimeVRPosition::LeftController,
	SlimeVRPosition::RightController,
	SlimeVRPosition::LeftFoot,
	SlimeVRPosition::RightFoot,
	SlimeVRPosition::LeftShoulder,
	SlimeVRPosition::RightShoulder,
	SlimeVRPosition::LeftElbow,
	SlimeVRPosition::RightElbow,
	SlimeVRPosition::LeftKnee,
	SlimeVRPosition::RightKnee,
	SlimeVRPosition::Waist,
	SlimeVRPosition::Chest
};

static constexpr const char* positionNames[(int)SlimeVRPosition::GenericController + 1] = {
	"None",
	"Waist",
	"LeftFoot",
	"RightFoot",
	"Chest",
	"LeftKnee",
	"RightKnee",
	"LeftElbow",
	"RightElbow",
	"LeftShoulder",
	"RightShoulder",
	"LeftHand",
	"RightHand",
	"LeftController",
	"RightController",
	"Head",
	"Neck",
	"Camera",
	"Keyboard",
	"HMD",
	"Beacon",
	"GenericController"
};

static constexpr const char* actions[(int)BodyPosition::BodyPosition_Count] = {
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

template<typename F>
std::optional<std::string> GetOpenVRString(F &&openvr_closure) {
	uint32_t size = openvr_closure(nullptr, 0);

	if (size == 0) {
		return std::nullopt;
	}

	std::string prop_value = std::string(size, '\0');
	uint32_t error = openvr_closure(prop_value.data(), size);
	prop_value.resize(size-1);

	if (error == 0) {
		return std::nullopt;
	}

	return prop_value;
}

VRActionHandle_t GetAction(const char* action_path) {
	VRActionHandle_t handle = k_ulInvalidInputValueHandle;
	EVRInputError error = (EVRInputError)VRInput()->GetActionHandle(action_path, &handle);
	if (error != VRInputError_None) {
		fmt::print("Error: Unable to get action handle '{}': {}", action_path, error);
		// consider exiting?
	}

	return handle;
}

class UniverseTranslation {
	public:
		// TODO: do we want to store this differently?
		vr::HmdVector3_t translation;
		float yaw;

		static UniverseTranslation parse(simdjson::ondemand::object &&obj) {
			UniverseTranslation res;
			int iii = 0;
			for (auto component: obj["translation"]) {
				if (iii > 2) {
					break; // TODO: 4 components in a translation vector? should this be an error?
				}
				res.translation.v[iii] = component.get_double();
				iii += 1;
			}
			res.yaw = obj["yaw"].get_double();

			return res;
		}
};

enum class TrackerState {
	DISCONNECTED,
	WAITING,
	RUNNING
};

struct TrackerInfo {
	std::string name = "";
	std::optional<std::string> serial = std::nullopt;
	SlimeVRPosition position = SlimeVRPosition::None;
	messages::TrackerStatus_Status status = messages::TrackerStatus_Status_DISCONNECTED;

	TrackerState state = TrackerState::DISCONNECTED;
	/// number of ticks since last detect or valid pose
	uint8_t connection_timeout = 0;
	/// number of ticks since NONE position was first detected
	uint8_t detect_timeout = 0;

	bool is_slimevr = false;
};

class Trackers {
private:
	TrackerInfo tracker_info[k_unMaxTrackedDeviceCount] = {};
	TrackedDevicePose_t poses[k_unMaxTrackedDeviceCount];

	std::set<TrackedDeviceIndex_t> current_trackers = {};
	//TrackedDeviceIndex_t current_trackers[k_unMaxTrackedDeviceCount];
	//uint32_t current_trackers_size = 0;

	SlimeVRBridge &bridge;
public:
	VRActiveActionSet_t actionSet;
	std::optional<std::pair<uint64_t, UniverseTranslation>> current_universe = std::nullopt;
private:
	ETrackingUniverseOrigin universe;
	VRActionHandle_t action_handles[(int)BodyPosition::BodyPosition_Count];

	Trackers(SlimeVRBridge &bridge, ETrackingUniverseOrigin universe): bridge(bridge), universe(universe) {}

	std::optional<std::string> GetStringProp(TrackedDeviceIndex_t index, ETrackedDeviceProperty prop) {
		if (index >= k_unMaxTrackedDeviceCount) {
			fmt::print("GetStringProp: Got invalid index {}!\n", index);
			return std::nullopt;
		}

		auto get_prop = [index, prop](char *prop_str, uint32_t passed_size) {
			auto system = VRSystem();
			vr::ETrackedPropertyError prop_error = TrackedProp_Success;
			uint32_t size = system->GetStringTrackedDeviceProperty(index, prop, prop_str, passed_size, &prop_error);

			if (size == 0 || (prop_error != TrackedProp_Success && prop_error != TrackedProp_BufferTooSmall)) {
				if (prop_error != TrackedProp_Success) {
					fmt::print("Error getting {}: IVRSystem::GetStringTrackedDeviceProperty({}): {}\n", size ? "data" : "size", prop, system->GetPropErrorNameFromEnum(prop_error));
				}

				return (uint32_t)0;
			}

			return size;
		};
		
		return GetOpenVRString(get_prop);
	}

	std::optional<std::string> GetLocalizedName(VRInputValueHandle_t handle, EVRInputStringBits flags) {
		std::string name = std::string(100, '\0');
		EVRInputError input_error = VRInput()->GetOriginLocalizedName(handle, name.data(), 100, flags | EVRInputStringBits::VRInputString_ControllerType);

		if (input_error != VRInputError_None && input_error != VRInputError_BufferTooSmall) {
			if (input_error != VRInputError_None) {
				fmt::print("Error getting data: IVRInput::GetOriginLocalizedName(): {}\n", input_error);
			}

			return std::nullopt;
		}

		name.resize(strlen(name.data()));

		return name;
	}

	std::optional<TrackedDeviceIndex_t> GetIndex(VRInputValueHandle_t value_handle) {
		InputOriginInfo_t info;
		EVRInputError error = VRInput()->GetOriginTrackedDeviceInfo(value_handle, &info, sizeof(info));
		if (error != EVRInputError::VRInputError_None) {
			fmt::print("Error: IVRInput::GetOriginTrackedDeviceInfo: {}\n", error);
			return std::nullopt;
		}

		if (info.trackedDeviceIndex >= k_unMaxTrackedDeviceCount) {
			fmt::print("GetIndex: Got invalid index {}!\n", info.trackedDeviceIndex);
			return std::nullopt;
		}

		return std::make_optional(info.trackedDeviceIndex);
	}

	void SetStatus(TrackedDeviceIndex_t index, messages::TrackerStatus_Status status_val, bool send_anyway) {
		if (index >= k_unMaxTrackedDeviceCount) {
			fmt::print("SetStatus: Got invalid index {}!\n", index);
			return;
		}

		auto info = tracker_info + index;

		if (info->is_slimevr) {
			return; // don't send information on slimes
		}

		if (info->status == status_val && !send_anyway) {
			return; // already up to date;
		}

		info->status = status_val;

		messages::ProtobufMessage message;
		messages::TrackerStatus *status = message.mutable_tracker_status();

		status->set_status(status_val);
		status->set_tracker_id(index);

		bridge.sendMessage(message);

		fmt::print("Device (Index {}) status: {} ({})\n", index, messages::TrackerStatus_Status_Name(status_val), status_val);
	}

	void Update(TrackedDeviceIndex_t index, bool just_connected) {
		if (index >= k_unMaxTrackedDeviceCount) {
			fmt::print("Update: Got invalid index {}!\n", index);
			return;
		}
		auto pose = poses[index];
		auto info = tracker_info + index;

		if (info->state != TrackerState::RUNNING) {
			return;
		}

		// if(pose.bDeviceIsConnected) {
		// 	info->connection_timeout = 0;
		// }

		if (info->is_slimevr) {
			return; // don't bother with slimes
		}

		if (pose.bPoseIsValid || pose.eTrackingResult == ETrackingResult::TrackingResult_Fallback_RotationOnly) {
			if (pose.eTrackingResult == ETrackingResult::TrackingResult_Fallback_RotationOnly) {
				SetStatus(index, messages::TrackerStatus_Status_OCCLUDED, just_connected);
			} else {
				SetStatus(index, messages::TrackerStatus_Status_OK, just_connected);
			}

			HmdQuaternion_t new_rotation = GetRotation(pose.mDeviceToAbsoluteTracking);
			HmdVector3_t new_position = GetPosition(pose.mDeviceToAbsoluteTracking);

			if (current_universe.has_value()) {
				auto trans = current_universe.value().second;
				new_position.v[0] += trans.translation.v[0];
				new_position.v[1] += trans.translation.v[1];
				new_position.v[2] += trans.translation.v[2];

				// rotate by quaternion w = cos(-trans.yaw / 2), x = 0, y = sin(-trans.yaw / 2), z = 0
				auto tmp_w = cos(-trans.yaw / 2);
				auto tmp_y = sin(-trans.yaw / 2);
				auto new_w = tmp_w * new_rotation.w - tmp_y * new_rotation.y;
				auto new_x = tmp_w * new_rotation.x + tmp_y * new_rotation.z;
				auto new_y = tmp_w * new_rotation.y + tmp_y * new_rotation.w;
				auto new_z = tmp_w * new_rotation.z - tmp_y * new_rotation.x;

				new_rotation.w = new_w;
				new_rotation.x = new_x;
				new_rotation.y = new_y;
				new_rotation.z = new_z;

				// rotate point on the xz plane by -trans.yaw radians
				// this is equivilant to the quaternion multiplication, after applying the double angle formula.
				float tmp_sin = sin(-trans.yaw);
				float tmp_cos = cos(-trans.yaw);
				auto pos_x = new_position.v[0] * tmp_cos + new_position.v[2] * tmp_sin;
				auto pos_z = new_position.v[0] * -tmp_sin + new_position.v[2] * tmp_cos;

				new_position.v[0] = pos_x;
				new_position.v[2] = pos_z;
			}

			// send our position message
			messages::ProtobufMessage message;
			messages::Position *position = message.mutable_position();
			position->set_x(new_position.v[0]);
			position->set_y(new_position.v[1]);
			position->set_z(new_position.v[2]);
			position->set_qw(new_rotation.w);
			position->set_qx(new_rotation.x);
			position->set_qy(new_rotation.y);
			position->set_qz(new_rotation.z);
			position->set_tracker_id(index);
			position->set_data_source(
				pose.eTrackingResult == ETrackingResult::TrackingResult_Fallback_RotationOnly
				? messages::Position_DataSource_IMU
				: messages::Position_DataSource_FULL
			);

			bridge.sendMessage(message);
		}
		
		// send status update on change, or if we just connected.
		if (!pose.bDeviceIsConnected) {
			SetStatus(index, messages::TrackerStatus_Status_DISCONNECTED, just_connected);
		} else if (!pose.bPoseIsValid) {
			if (pose.eTrackingResult == ETrackingResult::TrackingResult_Calibrating_OutOfRange) {
				SetStatus(index, messages::TrackerStatus_Status_OCCLUDED, just_connected);
			} else {
				SetStatus(index, messages::TrackerStatus_Status_ERROR, just_connected);
			}
		}
	}

	void SetPosition(TrackedDeviceIndex_t index, SlimeVRPosition position, bool send_anyway) {
		if (index >= k_unMaxTrackedDeviceCount) {
			fmt::print("SetPosition: Got invalid index {}!\n", index);
			return;
		}
		auto info = tracker_info + index;

		info->connection_timeout = 0;

		if (info->is_slimevr) {
			return; // don't send information on slimes
		}

		bool should_send = false;

		switch (info->state) {
			case TrackerState::DISCONNECTED:
				info->position = position;
				if (position == SlimeVRPosition::None) {
					info->state = TrackerState::WAITING;
					info->detect_timeout = 0;
					fmt::print("Waiting for role for \"{}\" with index {}\n", info->name, index);
				} else {
					should_send = true;
					info->state = TrackerState::RUNNING;
				}
				break;

			case TrackerState::WAITING:
				if (position != SlimeVRPosition::None || info->detect_timeout >= 100) {
					if (info->detect_timeout >= 100) {
						fmt::print("Role timeout reached for index {}.\n", index);
					}
					info->position = position;
					info->state = TrackerState::RUNNING;
					should_send = true;
				} else {
					// increment timeout
					info->detect_timeout += 1;
				}
				break;

			case TrackerState::RUNNING:
				if (position != SlimeVRPosition::None && position != info->position) {
					info->position = position;
					should_send = true;
				}
				break;
		}

		if (should_send || (send_anyway && info->state == TrackerState::RUNNING)) {
			messages::ProtobufMessage message;
			messages::TrackerAdded *added = message.mutable_tracker_added();
			added->set_tracker_id(index);
			added->set_tracker_role((int)info->position);
			added->set_tracker_name(info->name);
			if (info->serial.has_value()) {
				added->set_tracker_serial(info->serial.value());
			}

			bridge.sendMessage(message);

			// log it.
			fmt::print("Found device \"{}\" at {} ({}) with index {}\n", info->name, positionNames[(int)info->position], (int)info->position, index);
		}
	}

public:
	static std::optional<Trackers> Create(SlimeVRBridge &bridge, ETrackingUniverseOrigin universe);

	void Detect(bool just_connected, bool enable_hmd) {
		current_trackers.clear();
		uint32_t all_trackers_size = 0;
		TrackedDeviceIndex_t all_trackers[k_unMaxTrackedDeviceCount];

		// only detect the HMD if the user requests it.
		if (enable_hmd) {
			all_trackers_size += VRSystem()->GetSortedTrackedDeviceIndicesOfClass(TrackedDeviceClass_HMD, all_trackers, k_unMaxTrackedDeviceCount);
		}
		// detect controllers and trackers, regardless of role.
		all_trackers_size += VRSystem()->GetSortedTrackedDeviceIndicesOfClass(TrackedDeviceClass_Controller, all_trackers + all_trackers_size, k_unMaxTrackedDeviceCount - all_trackers_size);
		all_trackers_size += VRSystem()->GetSortedTrackedDeviceIndicesOfClass(TrackedDeviceClass_GenericTracker, all_trackers + all_trackers_size, k_unMaxTrackedDeviceCount - all_trackers_size);

		if (just_connected) {
			fmt::print("number of trackers: {}\n", all_trackers_size);
		}

		for (auto iii = 0; iii < all_trackers_size; ++iii) {
			auto index = all_trackers[iii];
			auto driver = this->GetStringProp(index, ETrackedDeviceProperty::Prop_TrackingSystemName_String);
			auto info = tracker_info + index;

			info->is_slimevr = (driver == "SlimeVR" || driver == "slimevr");

			// only write values once, to avoid overwriting good values later.
			if (info->name == "") {
				auto controller_type = this->GetStringProp(index, ETrackedDeviceProperty::Prop_ControllerType_String);
				if (controller_type.has_value()) {
					info->name = controller_type.value();
				} else {
					// uhhhhhhhhhhhhhhh
					info->name = fmt::format("Index{}", index);
				}
			}

			info->serial = this->GetStringProp(index, ETrackedDeviceProperty::Prop_SerialNumber_String);

			current_trackers.insert(index);

			SetPosition(index, SlimeVRPosition::None, just_connected);
		}

		// detect roles, more specific names
		auto input = VRInput();
		EVRInputError input_error = input->UpdateActionState(&actionSet, sizeof(VRActiveActionSet_t), 1);
		if (input_error != EVRInputError::VRInputError_None) {
			fmt::print("Error: IVRInput::UpdateActionState: {}\n", input_error);
			return;
		}

		for (unsigned int jjj = 0; jjj < (int)BodyPosition::BodyPosition_Count; ++jjj) {
			InputPoseActionData_t pose;
			input_error = input->GetPoseActionDataRelativeToNow(action_handles[jjj], universe, 0, &pose, sizeof(pose), 0);
			if (input_error != EVRInputError::VRInputError_None) {
				fmt::print("Error: IVRInput::GetPoseActionDataRelativeToNow: {}\n", input_error);
				continue;
			}

			if (pose.bActive) {
				std::optional<TrackedDeviceIndex_t> trackedDeviceIndex = GetIndex(pose.activeOrigin);
				if (!trackedDeviceIndex.has_value()) {
					// already printed a message about this in GetIndex, just continue.
					continue;
				}

				auto index = trackedDeviceIndex.value();

				// TODO: I feel like this 'only left+right hand' thing is going to bite us in the ass later with things that aren't index/vive trackers.
				// oh well.
				auto name = GetLocalizedName(
					pose.activeOrigin,
					(jjj == (int)BodyPosition::LeftHand || jjj == (int)BodyPosition::RightHand)
						? EVRInputStringBits::VRInputString_Hand
						: (EVRInputStringBits)0
				);
				if (name.has_value()) {
					tracker_info[index].name = name.value();
				}

				current_trackers.insert(index);

				SetPosition(index, positionIDs[jjj], just_connected);
			}
		}

		for (auto iii = 0; iii < k_unMaxTrackedDeviceCount; ++iii) {
			auto info = tracker_info + iii;

			if (info->state == TrackerState::DISCONNECTED) {
				continue;
			}

			if (info->connection_timeout >= 100) {
				fmt::print("Tracker connection timeout.\n");
				info->state = TrackerState::DISCONNECTED;
				SetStatus(iii, messages::TrackerStatus_Status_DISCONNECTED, just_connected);
				info->name = "";
				info->connection_timeout = 0;
			} else {
				info->connection_timeout += 1;
			}
		}
	}

	void Tick(bool just_connected) {
		VRSystem()->GetDeviceToAbsoluteTrackingPose(universe, 0, poses, k_unMaxTrackedDeviceCount);
		for (TrackedDeviceIndex_t index: current_trackers) {
			Update(index, just_connected);
		}
	}

	std::optional<InputDigitalActionData_t> HandleDigitalActionBool(VRActionHandle_t action_handle, std::optional<const char *> server_name = std::nullopt) {
		InputDigitalActionData_t action_data;
		EVRInputError input_error = VRInputError_None;

		input_error = VRInput()->GetDigitalActionData(action_handle, &action_data, sizeof(InputDigitalActionData_t), 0);
		if (input_error == EVRInputError::VRInputError_None) {
			constexpr bool falling_edge = false; // rising edge for now, making it easy to switch for now just in case.
			if (action_data.bChanged && (action_data.bState ^ falling_edge) && server_name.has_value()) {
				messages::ProtobufMessage message;
				messages::UserAction *userAction = message.mutable_user_action();
				userAction->set_name(server_name.value());

				fmt::print("Sending {} action\n", server_name.value());

				bridge.sendMessage(message);
			}

			return action_data;
		} else {
			fmt::print("Error: VRInput::GetDigitalActionData: {}\n", input_error);
			return {};
		}
	}
};

std::optional<Trackers> Trackers::Create(SlimeVRBridge &bridge, ETrackingUniverseOrigin universe){
	VRActionSetHandle_t action_set_handle;
	EVRInputError input_error;
	Trackers result(bridge, universe);

	std::string actionsFileName = Path_MakeAbsolute(actions_path, Path_StripFilename(Path_GetExecutablePath()));

	if ((input_error = VRInput()->SetActionManifestPath(actionsFileName.c_str())) != EVRInputError::VRInputError_None) {
		fmt::print("Error: IVRInput::SetActionManifectPath: {}\n", input_error);
		return std::nullopt;
	}

	if ((input_error = VRInput()->GetActionSetHandle("/actions/main", &action_set_handle)) != EVRInputError::VRInputError_None) {
		fmt::print("Error: VRInput::GetActionSetHandle: {}\n", input_error);
		return std::nullopt;
	}

	result.actionSet = {
		action_set_handle,
		k_ulInvalidInputValueHandle,
		k_ulInvalidActionSetHandle,
		0,
		0
	};

	for (unsigned int iii = 0; iii < (int)BodyPosition::BodyPosition_Count; ++iii) {
		result.action_handles[iii] = GetAction(actions[iii]);
	}

	return result;
}

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

static const std::unordered_map<std::string, std::pair<ETrackingUniverseOrigin, bool>> universe_map {
	{"seated", {ETrackingUniverseOrigin::TrackingUniverseSeated, false}},
	{"standing", {ETrackingUniverseOrigin::TrackingUniverseStanding, false}},
	{"raw", {ETrackingUniverseOrigin::TrackingUniverseRawAndUncalibrated, false}},
	{"static_standing", {ETrackingUniverseOrigin::TrackingUniverseRawAndUncalibrated, true}}
};
// default is static_standing
static constexpr std::pair<ETrackingUniverseOrigin, bool> universe_default = {ETrackingUniverseOrigin::TrackingUniverseRawAndUncalibrated, true};

// TEMP, cba to setup a proper header file.
void test_lto();

std::optional<UniverseTranslation> search_universe(simdjson::ondemand::parser &json_parser, uint64_t target) {
	uint32_t length = 0;
	VRChaperoneSetup()->ExportLiveToBuffer(nullptr, &length);

	// compile time check to ensure we're being sane, otherwise this would be a buffer overrun!
	static_assert(simdjson::SIMDJSON_PADDING >= 1, "simdjson doesn't specify enough padding for a trailing null byte!");
	// i'd say something about caching a padded string to avoid allocation
	// but if it's not *exactly* the same size we'd need to allocate anyway
	// because simdjson doesn't allow modifying the valid length.
	auto json = simdjson::padded_string(length - 1);

	if (!VRChaperoneSetup()->ExportLiveToBuffer(json.data(), &length)) {
		return std::nullopt;
	}

	simdjson::ondemand::document doc;
	try {
		doc = json_parser.iterate(json);

		for (simdjson::ondemand::object uni: doc["universes"]) {
			// TODO: universeID comes after the translation, would it be faster to unconditionally parse the translation?
			simdjson::ondemand::value elem = uni["universeID"];
			
			uint64_t parsed_universe;
			auto is_integer = elem.is_integer();
			if (!is_integer.error() && is_integer.value_unsafe()) {
				parsed_universe = elem.get_uint64();
			} else {
				parsed_universe = elem.get_uint64_in_string();
			}
			if (parsed_universe == target) {
				return UniverseTranslation::parse(uni["standing"].get_object().value());
			}
		}
	} catch (simdjson::simdjson_error& e) {
		std::string_view raw_token_view;
		if (!doc.raw_json_token().get(raw_token_view)) {
			fmt::print("Error while parsing steamvr universes: {}\nraw_token: |{}|\n", e.error(), raw_token_view);
		} else {
			fmt::print("Error while parsing steamvr universes: {}\n", e.error());
		}
		return std::nullopt;
	}

	return std::nullopt;
}

int main(int argc, char* argv[]) {
	GOOGLE_PROTOBUF_VERIFY_VERSION;

	// test_lto();

	args::ArgumentParser parser("Feeds controller/tracker data to SlimeVR Server.", "This program also parses arguments from a config file \"config.txt\" in the same directory as the executable. It is formatted as one line per option, and ignores characters on a line after a '#' character. Options passed on the command line are parsed after those read from the config file, and thus override options read from the config file.");
	args::HelpFlag help(parser, "help", "Display this help menu", {'h', "help"});
	args::CompletionFlag completion(parser, {"complete"});

	args::MapFlag<std::string, std::pair<ETrackingUniverseOrigin, bool>> universe(
		parser,
		"universe",
		"Tracking Universe. Possible values:\n"
		"  raw: raw/uncalibrated space\n"
		"  seated: seated universe\n"
		"  standing: standing universe\n"
		"  static_standing: standing universe unaffected by playspace movement (this matches slimevr driver, default)",
		{"universe"},
		universe_map,
		universe_default
	);
	args::ValueFlag<uint32_t> tps(parser, "tps", "Ticks per second. i.e. the number of times per second to send tracking information to slimevr server. Default is 100.", {"tps"}, 100);
	args::Flag enable_hmd(parser, "hmd", "Enabled sending the HMD position along with controller/tracker information.", {"hmd"});

	args::Group setup_group(parser, "Setup options", args::Group::Validators::AtMostOne);
	args::Flag install(setup_group, "install", "Installs the manifest and enables autostart. Used by the installer.", {"install"});
	args::Flag uninstall(setup_group, "uninstall", "Removes the manifest file.", {"uninstall"});

	std::string configFileName = Path_MakeAbsolute(config_path, Path_StripFilename(Path_GetExecutablePath()));
	std::ifstream configFile(configFileName);

	std::vector<std::string> args;

	for (std::string line; std::getline(configFile, line); ) {
		const auto comment_pos = line.find("#");
		if (comment_pos == 0) {
			continue; // line immediately starts with a comment.
		}
		const auto end = line.find_last_not_of(" \t", comment_pos - 1);
		if (end == std::string::npos) {
			continue; // line consists of only blank characters or comments.
		}
		const auto begin = line.find_first_not_of(" \t");
		if (begin == std::string::npos) {
			continue; // line consists of only blank characters. should be caught by the previous if statement, though.
		}

		line = line.substr(begin, end - begin + 1); // perform the actual trimming

		args.push_back(line);
	}

	// place command line arguments on the end, to override any arguments in the config file.
	for (int iii = 1; iii < argc; ++iii) {
		args.push_back(argv[iii]);
	}

	try {
		parser.Prog(argv[0]);
		parser.ParseArgs(args);
	} catch (args::Help) {
		std::cout << parser;
		return 0;
	} catch (args::ParseError e) {
		std::cerr << e.what() << std::endl;
		std::cerr << parser;
		return 1;
	} catch (args::ValidationError e) {
		std::cerr << e.what() << std::endl;
		std::cerr << parser;
		return 1;
	}

	if (install || uninstall) {
		return handle_setup(install);
	}

	fmt::print("SlimeVR-Feeder-App version {}\n\n", version);

	EVRInitError init_error = VRInitError_None;
	EVRInputError input_error = VRInputError_None;

	signal(SIGINT, handle_signal);

	std::unique_ptr<IVRSystem, decltype(&shutdown_vr)> system(VR_Init(&init_error, VRApplication_Overlay), &shutdown_vr);
	if (init_error != VRInitError_None) {
		system = nullptr;
		fmt::print("Unable to init VR runtime: {}\n", VR_GetVRInitErrorAsEnglishDescription(init_error));
		return EXIT_FAILURE;
	}

	// Ensure VR Compositor is available, otherwise getting poses causes a crash (openvr v1.3.22)
	if (!VRCompositor()) {
		std::cout << "Failed to initialize VR compositor!" << std::endl;
		return EXIT_FAILURE;
	}

	auto bridge = SlimeVRBridge::factory();
	auto tracking_universe = universe.Get().first;
	bool use_vrchaperone = universe.Get().second;
	std::optional<Trackers> maybe_trackers = Trackers::Create(*bridge, tracking_universe);
	if (!maybe_trackers.has_value()) {
		return EXIT_FAILURE;
	}

	Trackers trackers = maybe_trackers.value();

	VRActionHandle_t calibration_action = GetAction("/actions/main/in/request_calibration");
	VRActionHandle_t fast_reset_action = GetAction("/actions/main/in/fast_reset");

	//trackers.Detect(false);

	auto tick_ns = std::chrono::nanoseconds(1'000'000'000 / tps.Get());
	auto next_tick = std::chrono::duration_cast<std::chrono::nanoseconds>(
		std::chrono::high_resolution_clock::now().time_since_epoch()
	);

	bool overlay_was_open = false;

	auto json_parser = simdjson::ondemand::parser();

	// event loop
	while (!should_exit) {
		bool just_connected = bridge->runFrame();

		VREvent_t event;
		// each loop is now spaced apart, so let's process all events right now.
		while (system->PollNextEvent(&event, sizeof(event))) {
			switch (event.eventType) {
			case VREvent_Quit:
				return 0;

			// TODO: add more events, or remove some events?
			// case VREvent_TrackedDeviceActivated:
			// case VREvent_TrackedDeviceDeactivated:
			// case VREvent_TrackedDeviceRoleChanged:
			// case VREvent_TrackedDeviceUpdated:
			// case VREvent_DashboardDeactivated:
				// trackers.Detect(just_connected);
				// break;

			default:
				//fmt::print("Unhandled event: {}({})\n", system->GetEventTypeNameFromEnum((EVREventType)event.eventType), event.eventType);
				// I'm not relying on events to actually trigger anything right now, so don't bother printing anything.
				break;
			}
		}

		messages::ProtobufMessage recievedMessage;
		// TODO: I don't think there are any messages from the server that we care about at the moment, but let's make sure to not let the pipe fill up.
		bridge->getNextMessage(recievedMessage);

		// TODO: are there events we should be listening to in order to fire this?
		uint64_t universe = VRSystem()->GetUint64TrackedDeviceProperty(0, Prop_CurrentUniverseId_Uint64);
		if (use_vrchaperone && (!trackers.current_universe.has_value() || trackers.current_universe.value().first != universe)) {
			auto res = search_universe(json_parser, universe);
			if (res.has_value()) {
				trackers.current_universe.emplace(universe, res.value());
			}
		}

		// TODO: don't do this every loop, we really shouldn't need to.
		if (VROverlay()->IsDashboardVisible()) {
			if (!overlay_was_open) {
				fmt::print("Dashboard open, pausing detection.\n");
			}
			overlay_was_open = true;

			// we should still be updating the action state, to get DigitalActions while the dashboard is open.
			VRInput()->UpdateActionState(&trackers.actionSet, sizeof(VRActiveActionSet_t), 1);
		} else {
			if (overlay_was_open) {
				fmt::print("Dashboard closed, re-enabling tracker detection.\n");
			}
			overlay_was_open = false;
			trackers.Detect(just_connected, enable_hmd);
		}

		// TODO: rename these actions as appropriate, perhaps log them?
		trackers.HandleDigitalActionBool(calibration_action, { "reset" });
		trackers.HandleDigitalActionBool(fast_reset_action, { "fast_reset" });

		trackers.Tick(just_connected);

		next_tick += tick_ns;

		auto wait_ns = next_tick - std::chrono::duration_cast<std::chrono::nanoseconds>(
			std::chrono::high_resolution_clock::now().time_since_epoch()
		);

		if (wait_ns.count() > 0) {
			std::this_thread::sleep_for(wait_ns);
		} else {
			// I don't care if you want more TPS than the feeder can provide, I'm yielding to the OS anyway.
			// if this is really an issue for someone, they can open an issue.
			std::this_thread::yield();
		}
	}

	fmt::print("Exiting cleanly!\n");

	return 0;
}
