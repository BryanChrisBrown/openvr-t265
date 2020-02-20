//============ Copyright (c) Valve Corporation, All rights reserved. ============

#include <openvr_driver.h>
#include "driverlog.h"

#include <vector>
#include <thread>
#include <chrono>
#include <cmath>

#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>

#if defined( _WINDOWS )
#include <windows.h>
#endif

using namespace vr;


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
		// Declare RealSense pipeline, encapsulating the actual device and sensors
		rs2::pipeline pipe;
		// Create a configuration for configuring the pipeline with a non default profile
		rs2::config cfg;
		// Add pose stream
		cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
		// Start pipeline with chosen configuration
		pipe.start(cfg);

		// Main loop
		while (true)
		{
			// Wait for the next set of frames from the camera
			auto frames = pipe.wait_for_frames();
			// Get a frame from the pose stream
			auto f = frames.first_or_default(RS2_STREAM_POSE);
			// Cast the frame to pose_frame and get its data
			auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

			// Check to see if the tracker confidence is not failing.
			// If not failing provide pose.
			DriverPose_t pose = { 0 };
			pose.poseIsValid = pose_data.tracker_confidence != 0;
			pose.result = TrackingResult_Running_OK;
			pose.deviceIsConnected = true;

			// TO DO: Expose to vr settings/launcher
			pose.qWorldFromDriverRotation = HmdQuaternion_Init(1, 0, 0, 0);
			pose.qDriverFromHeadRotation = HmdQuaternion_Init(1, 0, 0, 0);

			// make the tracker go woosh up and down
			// std::chrono::milliseconds timeMs = std::chrono::duration_cast< std::chrono::milliseconds >(
			// 	std::chrono::system_clock::now().time_since_epoch()
			// );
			// double timeOffset = sin(timeMs.count() / 2000.0);
			// pose.vecPosition[1] = 1.0 + timeOffset / 4.0;

			// The reason we have to break everything up is because realsense uses structs 
			// while openvr uses arrays. The main difference is the types. 
			// The code below avoids having to write an additional helper function and is easier to read.
			pose.vecPosition[0] = pose_data.translation.x;
			pose.vecPosition[1] = pose_data.translation.y;
			pose.vecPosition[2] = pose_data.translation.z;
			pose.vecVelocity[0] = pose_data.velocity.x;
			pose.vecVelocity[1] = pose_data.velocity.y;
			pose.vecVelocity[2] = pose_data.velocity.z;
			pose.vecAngularVelocity[0] = pose_data.angular_velocity.x;
			pose.vecAngularVelocity[1] = pose_data.angular_velocity.y;
			pose.vecAngularVelocity[2] = pose_data.angular_velocity.z;
			pose.vecAngularAcceleration[0] = pose_data.angular_acceleration.x;
			pose.vecAngularAcceleration[1] = pose_data.angular_acceleration.y;
			pose.vecAngularAcceleration[2] = pose_data.angular_acceleration.z;
			pose.vecAcceleration[0] = pose_data.acceleration.x;
			pose.vecAcceleration[1] = pose_data.acceleration.y;
			pose.vecAcceleration[2] = pose_data.acceleration.z;
			pose.qRotation.w = pose_data.rotation.w;
			pose.qRotation.x = pose_data.rotation.x;
			pose.qRotation.y = pose_data.rotation.y;
			pose.qRotation.z = pose_data.rotation.z;

			if (*m_unObjectId != vr::k_unTrackedDeviceIndexInvalid)
			{
				vr::VRServerDriverHost()->TrackedDevicePoseUpdated(*m_unObjectId, pose, sizeof(DriverPose_t));

				// this is not implemented properly don't use
				// void WaitGetPoses ( VR_ARRAY_COUNT(unPoseArrayCount) TrackedDevicePose_t* pPoseArray, uint32_t unPoseArrayCount ) = 0;
			}
		}

		return EXIT_SUCCESS;
	}
	catch (const rs2::error & e)
	{
		DriverLog("RealSense error calling %s (%s): %s\n", e.get_failed_function(), e.get_failed_args(), e.what());
		return EXIT_FAILURE;
	}
	catch (const std::exception & e)
	{
		DriverLog("%s\n", e.what());
		return EXIT_FAILURE;
	}
}


// keys for use with the settings API
static const char* const k_pch_Sample_Section = "driver_t265";
static const char* const k_pch_Sample_SerialNumber_String = "serialNumber";
static const char* const k_pch_Sample_ModelNumber_String = "modelNumber";



//-----------------------------------------------------------------------------
// Purpose:This part of the code sets up the actual device as far as steamvr is concerned. (note that device type is determined by the CServerDriver_T265 (Currently line 256)
//-----------------------------------------------------------------------------
class CT265Driver : public vr::ITrackedDeviceServerDriver
{
public:
	CT265Driver()
	{
		m_unObjectId = vr::k_unTrackedDeviceIndexInvalid;
		m_ulPropertyContainer = vr::k_ulInvalidPropertyContainer;
		// TO DO: Plugin actual info
		m_sSerialNumber = "CTRL_1234";

		m_sModelNumber = "MyController";
	}

	virtual ~CT265Driver()
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

		// pose thread for realsense t-265
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
class CServerDriver_T265 : public IServerTrackedDeviceProvider
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
	CT265Driver* m_pTracker = nullptr;
};

CServerDriver_T265 g_serverDriverNull;


EVRInitError CServerDriver_T265::Init(vr::IVRDriverContext* pDriverContext)
{
	VR_INIT_SERVER_DRIVER_CONTEXT(pDriverContext);
	InitDriverLog(vr::VRDriverLog());

	m_pTracker = new CT265Driver();
	vr::VRServerDriverHost()->TrackedDeviceAdded(m_pTracker->GetSerialNumber().c_str(), vr::TrackedDeviceClass_GenericTracker, m_pTracker);

	return VRInitError_None;
}

void CServerDriver_T265::Cleanup()
{
	CleanupDriverLog();
	delete m_pTracker;
	m_pTracker = NULL;
}


void CServerDriver_T265::RunFrame()
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
