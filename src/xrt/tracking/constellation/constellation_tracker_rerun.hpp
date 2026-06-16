// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Internal header for constellation tracker usage of the constellation tracker rerun logging.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#pragma once

#include "xrt/xrt_config_build.h"

// @note In general usage, we don't use the internal header, rerun is an opt-in debugging feature, so we are OK using
//       internal APIs here to keep code clean and non-invasive. Please look at target_builder_rift.c instead for normal
//       usage of these APIs.
#include "t_constellation_tracker_internal.hpp"

#include <rerun.hpp>


namespace xrt::tracking::constellation {

const std::string rerun_recording_id = "constellation_tracking";

struct RerunContext
{
public: // Fields
	std::unique_ptr<rerun::RecordingStream> stream{};

public: // Methods
	RerunContext()
	{
		stream = std::make_unique<rerun::RecordingStream>(rerun_recording_id);
	}

	/*! Log a tracked frame to Rerun.
	 *
	 * All pose parameters must be in OpenXR coordinate space (Y-up). They will be converted to OpenCV space
	 * (Y-down) internally for logging.
	 *
	 * @param camera_sample Camera frame with blobs
	 * @param device_id Device being tracked
	 * @param led_model LED model in device-local OpenCV space
	 * @param calibration Camera intrinsics
	 * @param Txr_world_cam World → Camera pose (OpenXR space)
	 * @param Txr_cam_device Camera → Device pose (OpenXR space)
	 * @param Txr_world_device World → Device pose (OpenXR space)
	 * @param brightness Average brightness of matched blobs
	 */
	void
	LogTrackedFrame(const CameraSample &camera_sample,
	                t_constellation_device_id_t device_id,
	                const t_constellation_tracker_led_model &led_model,
	                const t_camera_calibration &calibration,
	                const xrt_pose &Txr_world_cam,
	                const xrt_pose &Txr_cam_device,
	                const xrt_pose &Txr_world_device,
	                float brightness);

private: // Methods
	void
	LogStaticScene(const CameraSample &camera_sample, const t_camera_calibration &calibration);

	void
	LogLedModel(const std::string &entity_name, const t_constellation_tracker_led_model &led_model);

	void
	LogBlobSet(const std::string &entity_name,
	           const CameraSample &camera_sample,
	           t_constellation_device_id_t device_id,
	           bool matched_only);

	void
	LogFrameMetrics(const CameraSample &camera_sample, t_constellation_device_id_t device_id, float brightness);
};

}; // namespace xrt::tracking::constellation
