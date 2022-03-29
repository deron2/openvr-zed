//============ Copyright (c) Valve Corporation, All rights reserved. ============
#include <openvr_driver.h>
#include "driverlog.h"

#include <vector>
#include <thread>
#include <chrono>
#include <cmath>

#include <sl/Camera.hpp>
#include <iostream>
#include <iomanip>

#include <windows.h>

using namespace vr;
using namespace sl;

#if defined(_WIN32)
#define HMD_DLL_EXPORT extern "C" __declspec( dllexport )
#define HMD_DLL_IMPORT extern "C" __declspec( dllimport )
#elif defined(__GNUC__) || defined(COMPILER_GCC) || defined(__APPLE__)
#define HMD_DLL_EXPORT extern "C" __attribute__((visibility("default")))
#define HMD_DLL_IMPORT extern "C"
#else
#error "Unsupported Platform."
#endif

inline HmdQuaternion_t HmdQuaternion_Init(double w, double x, double y, double z)
{
	HmdQuaternion_t quat;
	quat.w = w;
	quat.x = x;
	quat.y = y;
	quat.z = z;
	return quat;
}

int runPoseTracking(vr::TrackedDeviceIndex_t* m_unObjectId) {
	try
	{
		// Create a ZED camera object
		Camera zed;

		// Set configuration parameters
		InitParameters init_params;
		init_params.camera_resolution = RESOLUTION::HD720; // Use HD720 video mode (default fps: 60)
		init_params.coordinate_system = COORDINATE_SYSTEM::RIGHT_HANDED_Y_UP; // Use a right-handed Y-up coordinate system
		init_params.coordinate_units = UNIT::METER; // Set units in meters
		init_params.sensors_required = true;

		// Open the camera
		zed.open(init_params);
		// Enable positional tracking with default parameters
		PositionalTrackingParameters tracking_parameters;
		zed.enablePositionalTracking(tracking_parameters);

		// Track the camera position during 1000 frames
		int i = 0;
		Pose zed_pose;

		// Check if the camera is a ZED M and therefore if an IMU is available
		SensorsData sensor_data;

		while (true)
		{
			if (zed.grab() == ERROR_CODE::SUCCESS) {
				zed.getPosition(zed_pose, REFERENCE_FRAME::WORLD);

				// get the translation information
				auto zed_translation = zed_pose.getTranslation();

				//	Display the translation and timestamp
				//	DriverLog("\nTranslation: Tx: %.3f, Ty: %.3f, Tz: %.3f, Timestamp: %llu\n", zed_translation.tx,
				//	zed_translation.ty, zed_translation.tz, (long long unsigned int) zed_pose.timestamp.getNanoseconds());

				// get the orientation information
				auto zed_orientation = zed_pose.getOrientation();

				// Display the orientation quaternion
				DriverLog("Orientation: Ox: %.3f, Oy: %.3f, Oz: %.3f, Ow: %.3f\n", zed_orientation.ox,
					zed_orientation.oy, zed_orientation.oz, zed_orientation.ow);

				// Get IMU data
				zed.getSensorsData(sensor_data, TIME_REFERENCE::IMAGE);

				auto imu_orientation = sensor_data.imu.pose.getOrientation();

				// Filtered orientation quaternion
				DriverLog("IMU Orientation: Ox: %.3f, Oy: %.3f, Oz: %.3f, Ow: %.3f\n", imu_orientation.ox,
					imu_orientation.oy, imu_orientation.oz, imu_orientation.ow);

				DriverPose_t pose = { 0 };
				pose.poseIsValid = true;
				pose.result = TrackingResult_Running_OK;
				pose.deviceIsConnected = true;

				// TO DO: Expose to vr settings/launcher
				pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
				pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

				pose.vecPosition[0] = zed_translation.tx;
				pose.vecPosition[1] = zed_translation.ty;
				pose.vecPosition[2] = zed_translation.tz;

				pose.qRotation.w = imu_orientation.ow;
				pose.qRotation.x = imu_orientation.ox;
				pose.qRotation.y = imu_orientation.oy;
				pose.qRotation.z = imu_orientation.oz;

				if (*m_unObjectId != vr::k_unTrackedDeviceIndexInvalid)
				{
					vr::VRServerDriverHost()->TrackedDevicePoseUpdated(*m_unObjectId, pose, sizeof(DriverPose_t));
				}
			}
		}
		// Disable positional tracking and close the camera
		zed.disablePositionalTracking();
		zed.close();
		return EXIT_SUCCESS;
	}
	catch (const std::exception& e)
	{
		DriverLog("%s\n", e.what());
		return EXIT_FAILURE;
	}

}

// keys for use with the settings API
static const char* const k_pch_Sample_Section = "driver_zedm";
static const char* const k_pch_Sample_SerialNumber_String = "serialNumber";
static const char* const k_pch_Sample_ModelNumber_String = "modelNumber";

//-----------------------------------------------------------------------------
// Purpose:This part of the code sets up the actual device as far as SteamVR is concerned. (note that device type is determined by the CServerDriver_Zedm (Currently line 256)
//-----------------------------------------------------------------------------
class CZedmDriver : public vr::ITrackedDeviceServerDriver
{
public:
	CZedmDriver()
	{
		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
		m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;
		// TO DO: Plugin actual info
		m_sSerialNumber = "CTRL_1234";

		m_sModelNumber = "MyController";
	}

	virtual ~CZedmDriver()
	{
	}


	virtual EVRInitError Activate(vr::TrackedDeviceIndex_t unObjectId)
	{
		m_unObjectId = unObjectId;
		m_ulPropertyContainer = vr::VRProperties()->TrackedDeviceToPropertyContainer(m_unObjectId);

		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_ModelNumber_String, m_sModelNumber.c_str());
		vr::VRProperties()->SetStringProperty(m_ulPropertyContainer, Prop_RenderModelName_String, m_sModelNumber.c_str());

		// return a constant that's not 0 (invalid) or 1 (reserved for Oculus)
		vr::VRProperties()->SetUint64Property(m_ulPropertyContainer, Prop_CurrentUniverseId_Uint64, 27);

		// avoid "not fullscreen" warnings from vrmonitor |Fullscreen error still present
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_IsOnDesktop_Bool, false);

		// The Realsense Driver is intended to be a tracked device yall
		vr::VRProperties()->SetBoolProperty(m_ulPropertyContainer, Prop_NeverTracked_Bool, false);

		// our device is not a controller, it's a generic tracker | No Change upon commenting line out. | very confusing because at one point this did *something* maybe.
		vr::VRProperties()->SetInt32Property(m_ulPropertyContainer, Prop_ControllerRoleHint_Int32, TrackedControllerRole_OptOut);

		DriverLog("Driver has been initialized\n");

		// pose thread for zedm
		m_pPoseThread = new std::thread(runPoseTracking, &m_unObjectId);
		if (!m_pPoseThread)
		{
			DriverLog("Unable to create tracking thread\n");
			return VRInitError_Driver_Failed;
		}

		return VRInitError_None;
	}

	virtual void Deactivate()
	{
		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
	}

	virtual void EnterStandby()
	{
	}

	void* GetComponent(const char* pchComponentNameAndVersion)
	{
		// override this to add a component to a driver
		return NULL;
	}

	virtual void PowerOff()
	{
	}

	/** debug request from a client */
	virtual void DebugRequest(const char* pchRequest, char* pchResponseBuffer, uint32_t unResponseBufferSize)
	{
		if (unResponseBufferSize >= 1)
			pchResponseBuffer[0] = 0;
	}

	virtual DriverPose_t GetPose() // hook in the realsense here bois
	{
		DriverPose_t pose = { 0 };
		pose.poseIsValid = true;
		pose.result = TrackingResult_Running_OK;
		pose.deviceIsConnected = true;

		pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
		pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

		return pose;
	}

	void RunFrame()
	{
		// In a real driver, this should happen from some pose tracking thread.
		// The RunFrame interval is unspecified and can be very irregular if some other
		// driver blocks it for some periodic task.
		if (m_unObjectId != vr::k_unTrackedDeviceIndexInvalid)
		{
			// vr::VRServerDriverHost()->TrackedDevicePoseUpdated( m_unObjectId, GetPose(), sizeof( DriverPose_t ) );
		}
	}

	void ProcessEvent(const vr::VREvent_t& vrEvent)
	{

	}

	std::string GetSerialNumber() const { return m_sSerialNumber; }

private:
	vr::TrackedDeviceIndex_t m_unObjectId;
	vr::PropertyContainerHandle_t m_ulPropertyContainer;

	std::string m_sSerialNumber;
	std::string m_sModelNumber;
	std::thread* m_pPoseThread;
};

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
class CServerDriver_Zedm : public IServerTrackedDeviceProvider
{
public:
	virtual EVRInitError Init(vr::IVRDriverContext* pDriverContext);
	virtual void Cleanup();
	virtual const char* const* GetInterfaceVersions() { return vr::k_InterfaceVersions; }
	virtual void RunFrame();
	virtual bool ShouldBlockStandbyMode() { return false; }
	virtual void EnterStandby() {}
	virtual void LeaveStandby() {}

private:
	CZedmDriver* m_pTracker = nullptr;
};

CServerDriver_Zedm g_serverDriverNull;


EVRInitError CServerDriver_Zedm::Init(vr::IVRDriverContext* pDriverContext)
{
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
	InitDriverLog(vr::VRDriverLog());

	m_pTracker = new CZedmDriver();
	vr::VRServerDriverHost()->TrackedDeviceAdded(m_pTracker->GetSerialNumber().c_str(), vr::TrackedDeviceClass_GenericTracker, m_pTracker);

	return VRInitError_None;
}

void CServerDriver_Zedm::Cleanup()
{
	CleanupDriverLog();
	delete m_pTracker;
	m_pTracker = NULL;
}


void CServerDriver_Zedm::RunFrame()
{
	if (m_pTracker)
	{
		m_pTracker->RunFrame();
	}

	vr::VREvent_t vrEvent;
	while (vr::VRServerDriverHost()->PollNextEvent(&vrEvent, sizeof(vrEvent)))
	{
		if (m_pTracker)
		{
			m_pTracker->ProcessEvent(vrEvent);
		}
	}
}

//-----------------------------------------------------------------------------
// Purpose:
//-----------------------------------------------------------------------------
HMD_DLL_EXPORT void* HmdDriverFactory(const char* pInterfaceName, int* pReturnCode)
{
	if (0 == strcmp(IServerTrackedDeviceProvider_Version, pInterfaceName))
	{
		return &g_serverDriverNull;
	}

	if (pReturnCode)
		*pReturnCode = VRInitError_Init_InterfaceNotFound;

	return NULL;
}