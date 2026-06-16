// Copyright 2026, Beyley Cardellio
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Implementation of the rerun recorder logic for the constellation tracker.
 * @author Beyley Cardellio <ep1cm1n10n123@gmail.com>
 * @ingroup tracking
 */

#include "constellation_tracker_rerun.hpp"

#include "math/m_api.h"

#include <algorithm>
#include <array>
#include <cstdint>
#include <format>


using namespace xrt::auxiliary;
using namespace xrt::tracking::constellation;

// Anonymous namespace for internal functions.
namespace {

constexpr float AXIS_LENGTH = 0.03f;
constexpr float BLOB_RADIUS_PIXELS = 3.0f;

/*
 *
 * Helper functions
 *
 */

rerun::components::Translation3D
ToRerunTranslation(const xrt_vec3 &position)
{
	return rerun::components::Translation3D(position.x, position.y, position.z);
}

rerun::Rotation3D
ToRerunRotation(const xrt_quat &orientation)
{
	return rerun::Rotation3D(
	    rerun::datatypes::Quaternion::from_xyzw(orientation.x, orientation.y, orientation.z, orientation.w));
}

rerun::Transform3D
ToRerunTransform(const xrt_pose &pose, bool from_parent = true)
{
	auto transform = rerun::Transform3D()
	                     .with_translation(ToRerunTranslation(pose.position))
	                     .with_rotation(ToRerunRotation(pose.orientation));

	if (from_parent) {
		transform = std::move(transform).with_relation(rerun::components::TransformRelation::ParentFromChild);
	}

	return transform;
}

rerun::components::Color
ToBlobColor(float brightness, bool matched)
{
	float clamped = std::clamp(brightness, 0.0f, 1.0f);
	uint8_t intensity = static_cast<uint8_t>(clamped * 255.0f);
	if (matched) {
		return rerun::components::Color(intensity / 4, intensity, 64, 255);
	}

	return rerun::components::Color(intensity, intensity, intensity, 255);
}

rerun::Pinhole
MakePinhole(const t_camera_calibration &calibration)
{
	// Construct the 3x3 intrinsic matrix in column-major order.
	// Row-major K = [fx 0 cx; 0 fy cy; 0 0 1] becomes flat columns [fx 0 0, 0 fy 0, cx cy 1].
	std::array<float, 9> image_from_camera = {
	    static_cast<float>(calibration.intrinsics[0][0]),
	    0.0f,
	    0.0f,
	    0.0f,
	    static_cast<float>(calibration.intrinsics[1][1]),
	    0.0f,
	    static_cast<float>(calibration.intrinsics[0][2]),
	    static_cast<float>(calibration.intrinsics[1][2]),
	    1.0f,
	};

	return rerun::Pinhole(rerun::components::PinholeProjection(image_from_camera))
	    .with_resolution(calibration.image_size_pixels.w, calibration.image_size_pixels.h)
	    .with_image_plane_distance(0.2f);
}

/*
 *
 * Common entity names
 *
 */

std::string
GetCameraEntityName(size_t mosaic_idx, size_t camera_idx)
{
	return std::format("cameras/{}/{}", mosaic_idx, camera_idx);
}

std::string
GetDeviceEntityName(t_constellation_device_id_t device_id)
{
	return std::format("devices/{}", device_id);
}

std::string
GetWorldEntityName()
{
	return "world";
}

std::string
GetTimelineName(size_t mosaic_idx, size_t camera_idx)
{
	return std::format("{}/tracked_pose", GetCameraEntityName(mosaic_idx, camera_idx));
}

std::string
GetTimelineName(const CameraSample &camera_sample)
{
	return GetTimelineName(camera_sample.mosaic_index, camera_sample.camera_index);
}

std::string
GetWorldCameraEntityName(size_t mosaic_idx, size_t camera_idx)
{
	return std::format("{}/{}", GetWorldEntityName(), GetCameraEntityName(mosaic_idx, camera_idx));
}

std::string
GetCameraImageEntityName(size_t mosaic_idx, size_t camera_idx)
{
	return std::format("{}/image", GetWorldCameraEntityName(mosaic_idx, camera_idx));
}

std::string
GetWorldDeviceEntityName(t_constellation_device_id_t device_id)
{
	return std::format("{}/{}", GetWorldEntityName(), GetDeviceEntityName(device_id));
}

std::string
GetCameraTrackedDeviceEntityName(const CameraSample &camera_sample, t_constellation_device_id_t device_id)
{
	return std::format("{}/tracked_devices/{}",
	                   GetWorldCameraEntityName(camera_sample.mosaic_index, camera_sample.camera_index), device_id);
}

} // namespace

namespace xrt::tracking::constellation {

/*
 *
 * Private methods
 *
 */

void
RerunContext::LogStaticScene(const CameraSample &camera_sample, const t_camera_calibration &calibration)
{
	std::string camera_entity = GetWorldCameraEntityName(camera_sample.mosaic_index, camera_sample.camera_index);
	std::string camera_image_entity =
	    GetCameraImageEntityName(camera_sample.mosaic_index, camera_sample.camera_index);

	this->stream->log_static("/", rerun::ViewCoordinates::RIGHT_HAND_Y_DOWN);
	this->stream->log_static(GetWorldEntityName(), rerun::TransformAxes3D(AXIS_LENGTH));
	this->stream->log_static(camera_entity + "/axes", rerun::TransformAxes3D(AXIS_LENGTH));
	this->stream->log_static(camera_image_entity, MakePinhole(calibration));
}

void
RerunContext::LogLedModel(const std::string &entity_name, const t_constellation_tracker_led_model &led_model)
{
	std::vector<rerun::Position3D> positions;
	std::vector<rerun::Radius> radii;
	std::vector<rerun::components::Text> labels;
	positions.reserve(led_model.led_count);
	radii.reserve(led_model.led_count);
	labels.reserve(led_model.led_count);

	for (size_t i = 0; i < led_model.led_count; i++) {
		const t_constellation_tracker_led &led = led_model.leds[i];
		positions.emplace_back(led.position.x, led.position.y, led.position.z);
		radii.emplace_back(led.radius_m);
		labels.emplace_back(std::to_string(led.id));
	}

	this->stream->log_static(entity_name + "/axes", rerun::TransformAxes3D(AXIS_LENGTH));
	this->stream->log_static(      //
	    entity_name + "/leds",     //
	    rerun::Points3D(positions) //
	        .with_radii(radii)     //
	        .with_labels(labels)   //
	);                             //
}

void
RerunContext::LogBlobSet(const std::string &entity_name,
                         const CameraSample &camera_sample,
                         t_constellation_device_id_t device_id,
                         bool matched_only)
{
	std::vector<rerun::Position2D> positions;
	std::vector<rerun::Radius> radii;
	std::vector<rerun::components::Color> colors;
	std::vector<rerun::components::Text> labels;
	positions.reserve(camera_sample.blob_count);
	radii.reserve(camera_sample.blob_count);
	colors.reserve(camera_sample.blob_count);
	labels.reserve(camera_sample.blob_count);

	for (uint32_t i = 0; i < camera_sample.blob_count; i++) {
		const t_blob &blob = camera_sample.blobs[i];
		bool matched = blob.matched_device_id == device_id;
		if (matched_only && !matched) {
			continue;
		}
		if (!matched_only && matched) {
			continue;
		}

		positions.emplace_back(blob.center.x, blob.center.y);
		float radius = std::max(std::max(blob.size.x, blob.size.y) * 0.5f, BLOB_RADIUS_PIXELS);
		radii.emplace_back(rerun::Radius::ui_points(radius));
		colors.emplace_back(ToBlobColor(blob.brightness, matched));

		if (matched) {
			labels.emplace_back(std::format("LED {}", blob.matched_device_led_id));
		} else {
			labels.emplace_back(std::format("blob {}", blob.blob_id));
		}
	}

	this->stream->log(             //
	    entity_name,               //
	    rerun::Points2D(positions) //
	        .with_radii(radii)     //
	        .with_colors(colors)   //
	        .with_labels(labels)   //
	);                             //
}

void
RerunContext::LogFrameMetrics(const CameraSample &camera_sample,
                              t_constellation_device_id_t device_id,
                              float brightness)
{
	uint32_t matched_blob_count = 0;
	for (uint32_t i = 0; i < camera_sample.blob_count; i++) {
		if (camera_sample.blobs[i].matched_device_id == device_id) {
			matched_blob_count++;
		}
	}

	std::string device_metrics = GetWorldDeviceEntityName(device_id) + "/metrics";
	std::string camera_metrics =
	    GetWorldCameraEntityName(camera_sample.mosaic_index, camera_sample.camera_index) + "/metrics";

	this->stream->log(device_metrics + "/brightness", rerun::Scalars(brightness));
	this->stream->log(device_metrics + "/matched_blob_count",
	                  rerun::Scalars(static_cast<float>(matched_blob_count)));
	this->stream->log(camera_metrics + "/blob_count", rerun::Scalars(static_cast<float>(camera_sample.blob_count)));
	this->stream->log(camera_metrics + "/matched_blob_count",
	                  rerun::Scalars(static_cast<float>(matched_blob_count)));
}

/*
 *
 * Public methods
 *
 */

void
RerunContext::LogTrackedFrame(const CameraSample &camera_sample,
                              t_constellation_device_id_t device_id,
                              const t_constellation_tracker_led_model &led_model,
                              const t_camera_calibration &calibration,
                              const xrt_pose &Txr_world_cam,
                              const xrt_pose &Txr_cam_device,
                              const xrt_pose &Txr_world_device,
                              float brightness)
{
	std::string timeline_name = GetTimelineName(camera_sample);
	std::string camera_entity = GetWorldCameraEntityName(camera_sample.mosaic_index, camera_sample.camera_index);
	std::string camera_device_entity = GetCameraTrackedDeviceEntityName(camera_sample, device_id);
	std::string world_device_entity = GetWorldDeviceEntityName(device_id);
	std::string camera_image_entity =
	    GetCameraImageEntityName(camera_sample.mosaic_index, camera_sample.camera_index);

	// Convert all poses from OpenXR (Y-up) to OpenCV (Y-down) coordinate space
	xrt_pose Tcv_world_cam;
	xrt_pose Tcv_cam_device;
	xrt_pose Tcv_world_device;
	math_pose_convert_opencv(&Txr_world_cam, &Tcv_world_cam);
	math_pose_convert_opencv(&Txr_cam_device, &Tcv_cam_device);
	math_pose_convert_opencv(&Txr_world_device, &Tcv_world_device);

	this->LogStaticScene(camera_sample, calibration);
	this->stream->set_time_timestamp_nanos_since_epoch(timeline_name, camera_sample.timestamp_ns);

	this->stream->log(camera_entity, ToRerunTransform(Tcv_world_cam));
	this->stream->log(world_device_entity, ToRerunTransform(Tcv_world_device));
	this->stream->log(camera_device_entity, ToRerunTransform(Tcv_cam_device));

	// Log LED model only under the world device entity (primary representation).
	// The camera's view inherits this through the transform hierarchy.
	this->LogLedModel(world_device_entity, led_model);
	this->LogBlobSet(camera_image_entity + "/matched_blobs", camera_sample, device_id, true);
	this->LogBlobSet(camera_image_entity + "/other_blobs", camera_sample, device_id, false);
	this->LogFrameMetrics(camera_sample, device_id, brightness);
}

}; // namespace xrt::tracking::constellation
