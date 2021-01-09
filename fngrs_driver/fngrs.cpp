#include "openvr_driver.h"
#include "bones.h"
#include "driverlog.h"
#include <thread>
#include <atomic>
#include <chrono>

#include "serial_port.h"
#include "quat_utils.h"

#include <iostream>
#include <sstream>
#include <string>
#include <vector>
#include <iterator>
using namespace vr;
using namespace std;

#define DEVICE_NAME "fngrs"

static const char* device_manufacturer = "danwillm";
static const char* device_controller_type = DEVICE_NAME;
static const char* device_model_number = DEVICE_NAME "1";
static const char* right_controller_serial = "FNGRS1";
static const char* left_controller_serial = "FNGRS2";
static const char* device_render_model_name = "{" DEVICE_NAME "}/rendermodels/" DEVICE_NAME;
static const char* device_input_profile_path = "{" DEVICE_NAME "}/input/" DEVICE_NAME "_profile.json";

static const float c_x_offset = -0.07f; //- forward
static const float c_y_offset = -0.08f; //+ up
static const float c_z_offset = 0.07f; //+ right

//Set appropriate COM ports here (will be included in settings soon)
static const char* c_right_arduino_port = "\\\\.\\COM9";
static const char* c_left_arduino_port = "\\\\.\\COM8";

//Indexes for components - anything that you can bind in the steamvr bindings dashboard
static const enum ComponentIndex : int {
	JOYSTICK_X = 0,
	JOYSTICK_Y = 1,
	JOYSTICK_BTN = 2,
	BTN_TRIGGER = 3,
	BTN_A = 4
};
/*
	Main class for fngrs
*/
class Fngrs : public ITrackedDeviceServerDriver
{
public:
	uint32_t m_id;
	DriverPose_t m_pose;
	VRInputComponentHandle_t m_skeleton;
	ETrackedControllerRole m_role;
	string m_serial_number;
	vr::HmdVector3_t m_offset_vector;

	bool m_found_controller;

	atomic<bool> m_active;

	thread m_pose_thread;

	atomic<bool> m_found_arduino;

	thread m_skeletal_thread;

	int m_controller_id;

	Serial* m_arduino_serial;

	char receivedString[MAX_DATA_LENGTH];

	int m_last_component_value[6] = { 0,0,0,0,0,0 };

	int m_frame_count;

	VRInputComponentHandle_t m_h_component_values[8];

	Fngrs()
		: m_found_controller(false),
		m_active(false),
		m_found_arduino(false),
		m_frame_count(0)
	{}
	/*
		Initialise a controller
	*/
	void Init(ETrackedControllerRole role)
	{
		m_role = role;
		DebugDriverLog("This role is %i", role);
	}
	EVRInitError Activate(uint32_t unObjectId) override
	{
		m_id = unObjectId;
		m_pose = { 0 };
		m_pose.poseIsValid = true;
		m_pose.result = TrackingResult_Running_OK;
		m_pose.deviceIsConnected = true;
		m_pose.qDriverFromHeadRotation.w = 1;
		m_pose.qWorldFromDriverRotation.w = 1;
		m_pose.poseTimeOffset = -0.010f;
		const char* serial_number = m_role == TrackedControllerRole_RightHand ? "FNGRS1" : "FNGRS2";

		PropertyContainerHandle_t props = VRProperties()->TrackedDeviceToPropertyContainer(m_id);
		VRProperties()->SetStringProperty(props, Prop_SerialNumber_String, serial_number);
		VRProperties()->SetStringProperty(props, Prop_ModelNumber_String, device_model_number);
		//VRProperties()->SetStringProperty(props, Prop_RenderModelName_String, device_render_model_name);
		VRProperties()->SetStringProperty(props, Prop_ManufacturerName_String, device_manufacturer);
		VRProperties()->SetInt32Property(props, Prop_ControllerRoleHint_Int32, m_role == TrackedControllerRole_RightHand ? TrackedControllerRole_RightHand : TrackedControllerRole_LeftHand);
		VRProperties()->SetInt32Property(props, Prop_DeviceClass_Int32, (int32_t)TrackedDeviceClass_Controller);
		VRProperties()->SetInt32Property(props, Prop_ControllerHandSelectionPriority_Int32, (int32_t)2147483647);
		VRProperties()->SetStringProperty(props, Prop_InputProfilePath_String, device_input_profile_path);
		VRProperties()->SetStringProperty(props, Prop_ControllerType_String, device_controller_type);

		//Setup joystick
		vr::VRDriverInput()->CreateScalarComponent(props, "/input/joystick/x", &m_h_component_values[ComponentIndex::JOYSTICK_X], VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);
		vr::VRDriverInput()->CreateScalarComponent(props, "/input/joystick/y", &m_h_component_values[ComponentIndex::JOYSTICK_Y], VRScalarType_Absolute, VRScalarUnits_NormalizedTwoSided);

		//Setup buttons
		vr::VRDriverInput()->CreateBooleanComponent(props, "/input/joystick/click", &m_h_component_values[ComponentIndex::JOYSTICK_BTN]);


		vr::VRDriverInput()->CreateBooleanComponent(props, "/input/A/click", &m_h_component_values[ComponentIndex::BTN_A]);

		//TODO: this should be a scalar component, so that users can set their own grip poses, but for now, it's a boolean, as we need to figure out the best way for deciding the value of each finger.
		vr::VRDriverInput()->CreateBooleanComponent(props, "/input/grip/click", &m_h_component_values[6]);

		//This haptic component doesn't do anything (at least, yet), but some games require it to be bound to something, so just create a component for them to use.
		vr::VRDriverInput()->CreateHapticComponent(props, "output/haptic", &m_h_component_values[7]);

		if (m_role == TrackedControllerRole_RightHand) {
			vr::VRDriverInput()->CreateBooleanComponent(props, "/input/trigger/click", &m_h_component_values[ComponentIndex::BTN_TRIGGER]);

			m_offset_vector = { c_x_offset, c_y_offset, c_z_offset };

			vr::EVRInputError error = VRDriverInput()->CreateSkeletonComponent(props,
				"/input/skeleton/right",
				"/skeleton/hand/right",
				"/pose/raw",
				VRSkeletalTracking_Partial,
				right_fist_pose,
				NUM_BONES,
				&m_skeleton);
		}
		else {
			vr::VRDriverInput()->CreateBooleanComponent(props, "/input/system/click", &m_h_component_values[ComponentIndex::BTN_TRIGGER]);

			m_offset_vector = { -c_x_offset, c_y_offset, c_z_offset };

			vr::EVRInputError error = VRDriverInput()->CreateSkeletonComponent(props,
				"/input/skeleton/left",
				"/skeleton/hand/left",
				"/pose/raw",
				VRSkeletalTracking_Partial,
				left_fist_pose,
				NUM_BONES,
				&m_skeleton);

		}

		m_active = true;

		m_pose_thread = std::thread(&Fngrs::UpdatePoseThread, this);

		m_skeletal_thread = std::thread(&Fngrs::ParseSerialThread, this);

		return VRInitError_None;
	}
	DriverPose_t GetPose() override { return m_pose; }
	void Deactivate() override
	{
		if (m_active)
		{
			m_active = false;
			m_pose_thread.join();
			m_skeletal_thread.join();
		}
		m_arduino_serial->~Serial();

		delete m_arduino_serial;
	}

	void EnterStandby() override {}
	void* GetComponent(const char* pchComponentNameAndVersion) override
	{
		DebugDriverLog("GetComponent called for %s", pchComponentNameAndVersion);
		return nullptr;
	}

	void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize) override {}

	// update the controller position based on tracking system.
	void UpdateControllerPose()
	{
		if (!m_found_controller) {

			m_controller_id = FindController(m_role == TrackedControllerRole_RightHand);

			m_found_controller = true;

			DebugDriverLog("Found controller");
		}
		else {
			TrackedDevicePose_t hmd_pose[10];
			VRServerDriverHost()->GetRawTrackedDevicePoses(0, hmd_pose, 10);

			//Make sure that the pose is valid, if not, use a different way of tracking them
			if (hmd_pose[m_controller_id].bPoseIsValid)
			{
				vr::HmdMatrix34_t matrix = hmd_pose[m_controller_id].mDeviceToAbsoluteTracking;

				vr::HmdMatrix33_t rotation_matrix = Get33Matrix(matrix);

				vr::HmdVector3_t vector_offset = GetVectorOffset(rotation_matrix, m_offset_vector);

				m_pose.vecPosition[0] = hmd_pose[m_controller_id].mDeviceToAbsoluteTracking.m[0][3] + vector_offset.v[0];
				m_pose.vecPosition[1] = hmd_pose[m_controller_id].mDeviceToAbsoluteTracking.m[1][3] + vector_offset.v[1];
				m_pose.vecPosition[2] = hmd_pose[m_controller_id].mDeviceToAbsoluteTracking.m[2][3] + vector_offset.v[2]; //- forward


				vr::HmdQuaternion_t controller_rotation = GetRotation(matrix);
				vr::HmdQuaternion_t offset_quaternion = QuaternionFromAngle(1, 0, 0, DegToRad(-45));

				//merge rotation
				m_pose.qRotation = MultiplyQuaternion(controller_rotation, offset_quaternion);

				m_pose.result = TrackingResult_Running_OK;

				m_pose.vecAngularVelocity[0] = hmd_pose[m_controller_id].vAngularVelocity.v[0];
				m_pose.vecAngularVelocity[1] = hmd_pose[m_controller_id].vAngularVelocity.v[1];
				m_pose.vecAngularVelocity[2] = hmd_pose[m_controller_id].vAngularVelocity.v[2];

				m_pose.vecVelocity[0] = hmd_pose[m_controller_id].vVelocity.v[0];
				m_pose.vecVelocity[1] = hmd_pose[m_controller_id].vVelocity.v[1];
				m_pose.vecVelocity[2] = hmd_pose[m_controller_id].vVelocity.v[2];

				//set required properties
				m_pose.deviceIsConnected = true;

				m_pose.poseIsValid = true;

				//set the pose
				VRServerDriverHost()->TrackedDevicePoseUpdated(m_id, m_pose, sizeof(DriverPose_t));
			}
		}
	}

	int FindController(bool right_hand) {
		//This code is currently broken, and will be replaced with a more dynamic solution later.

		//int controller_id = -1;

		//vr::CVRPropertyHelpers* props = vr::VRProperties();

		//for (uint32_t i = 0; i < vr::k_unMaxTrackedDeviceCount; i++)
		//{
		//	vr::PropertyContainerHandle_t container = props->TrackedDeviceToPropertyContainer(i);
		//	vr::ETrackedPropertyError err;
		//	int32_t device_class = props->GetInt32Property(container, vr::ETrackedDeviceProperty::Prop_DeviceClass_Int32, &err);

		//	int32_t device_type = props->GetInt32Property(container, vr::ETrackedDeviceProperty::Prop_ControllerRoleHint_Int32, &err); //find if the device is right or left controller
		//	string device_maker = props->GetStringProperty(container, vr::ETrackedDeviceProperty::Prop_ManufacturerName_String, &err);
		//	string serial = props->GetStringProperty(container, vr::ETrackedDeviceProperty::Prop_SerialNumber_String, &err);
		//	DebugDriverLog("Found a controller: %c %i", serial.c_str(), device_type);
		//	if (serial != right_controller_serial || serial != left_controller_serial) {
		//		if (right_hand && device_type == TrackedControllerRole_RightHand) {
		//			return i;
		//		}
		//		else if (!right_hand && device_type == TrackedControllerRole_LeftHand) {
		//			return i;
		//		}
		//	}
		//}
		//return controller_id;

		return right_hand ? 4 : 3;
	}

	void ParseSerial() {
		if (!m_found_arduino) {
			m_arduino_serial = new Serial(m_role == TrackedControllerRole_RightHand ? c_right_arduino_port : c_left_arduino_port);

			if (m_arduino_serial->IsConnected()) {
				DebugDriverLog("Left arduino is connected");
			}

			m_found_arduino = true;

			this_thread::sleep_for(chrono::milliseconds(1500));

			DebugDriverLog("Purging buffer");

			m_arduino_serial->PurgeBuffer();
		}

		if (m_arduino_serial->IsConnected()) {
			//Read the data into the buffer
			int readResult = m_arduino_serial->ReadData(receivedString, MAX_DATA_LENGTH);

			receivedString[MAX_DATA_LENGTH - 1] = '\00';
			if (readResult != 0) {

				string str = receivedString;
				std::string buf;
				std::stringstream ss(str);

				std::vector<std::string> tokens;

				while (getline(ss, buf, '&'))
					tokens.push_back(buf);

				//Index at which to update the bones, this starts at the root index finger bone.
				int bone_index = 6;
				int btn_index = 0;
				int current_index = 0;

				vr::VRBoneTransform_t hand_pose[NUM_BONES];
				
				//Copy our open hand poses into an array that we will manipulate
				std::copy(std::begin(m_role == TrackedControllerRole_RightHand ? right_open_hand_pose : left_open_hand_pose), std::end(m_role == TrackedControllerRole_RightHand ? right_open_hand_pose : left_open_hand_pose), std::begin(hand_pose));

				int fingers_in_fist = 0;

				for (std::string& it : tokens) {
					try {
						int result = std::stoi(it);
						//Skeletal data
						if (current_index < 4) {
							if (m_frame_count % 100 == 0) {
								DebugDriverLog("%i", result);
							}
							ComputeBoneTransform(hand_pose, (float)result/1000.0f, bone_index, m_role == TrackedControllerRole_RightHand);

							fingers_in_fist += result;
							bone_index += 5;
						}
						else {
							//Button data
							if (m_last_component_value[btn_index] != result) {
								if (btn_index == 0 || btn_index == 1) {
									float adjusted_result = -((float)result - 512.0f) / 512.0f;

									VRDriverInput()->UpdateScalarComponent(m_h_component_values[btn_index], adjusted_result, 0);
								}
								else {
									DebugDriverLog("Update received! index: %i, value: %i", btn_index, result);
									VRDriverInput()->UpdateBooleanComponent(m_h_component_values[btn_index], result, 0);
								}
								m_last_component_value[btn_index] = result;
							}
							btn_index++;
						}
					}
					catch (const std::exception& e) {
						DebugDriverLog("Exception caught while trying to convert to int. Skipping...");
					}
					current_index += 1;
				}

				VRDriverInput()->UpdateSkeletonComponent(m_skeleton, VRSkeletalMotionRange_WithoutController, hand_pose, NUM_BONES);
				VRDriverInput()->UpdateSkeletonComponent(m_skeleton, VRSkeletalMotionRange_WithController, hand_pose, NUM_BONES);

				if ((fingers_in_fist/4) > 450) {
					DebugDriverLog("Fingers are in fist.");
					VRDriverInput()->UpdateBooleanComponent(m_h_component_values[6], 1, 0);
				}
				else {
					VRDriverInput()->UpdateBooleanComponent(m_h_component_values[6], 0, 0);
				}
			}
		}
	}

	void UpdatePoseThread() {

		while (m_active) {
			UpdateControllerPose();
			this_thread::sleep_for(chrono::milliseconds(5));
		}
	}

	void ParseSerialThread() {

		while (m_active) {
			ParseSerial();
			this_thread::sleep_for(chrono::milliseconds(5));
			m_frame_count++;
		}
	}
};

class DeviceProvider : public IServerTrackedDeviceProvider
{
public:
	Fngrs m_right_device;
	Fngrs m_left_device;

	EVRInitError Init(IVRDriverContext* pDriverContext) override
	{
		VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
		InitDriverLog(vr::VRDriverLog());

		m_right_device.Init(TrackedControllerRole_RightHand);
		m_left_device.Init(TrackedControllerRole_LeftHand);

		bool right_controler_activated = VRServerDriverHost()->TrackedDeviceAdded(right_controller_serial, TrackedDeviceClass_Controller, &m_right_device);
		bool left_controller_activated = VRServerDriverHost()->TrackedDeviceAdded(left_controller_serial, TrackedDeviceClass_Controller, &m_left_device);

		return VRInitError_None;
	}

	void Cleanup() override {}
	const char* const* GetInterfaceVersions() override
	{
		return k_InterfaceVersions;
	}
	void RunFrame() override {}
	bool ShouldBlockStandbyMode() override { return false; }
	void EnterStandby() override {}
	void LeaveStandby() override {}
} g_device_provider;

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C" 
#else
#error "Unsupported Platform."
#endif

////////////////////////////////////////////////////////////////////////////////////
// HmdDriverFactory : DLL entrypoint called by vrserver
////////////////////////////////////////////////////////////////////////////////////
HMD_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode)
{
	if (0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName))
	{
		return &g_device_provider;
	}
	DriverLog("HmdDriverFactory called for %s", pInterfaceName);
	return nullptr;
}
