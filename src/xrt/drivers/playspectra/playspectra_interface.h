// Copyright 2026, PlaySpectra.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Interface to the PlaySpectra virtual device driver.
 *
 * PlaySpectra は「VR版Playwright」の Monado Adapter。仮想 HMD(将来はコントローラ)を
 * Monado の正規デバイス経路に載せ、外部 PlaySpectra Server から NDJSON の VirtualDeviceState
 * (playspectra-device-core-spec.md) を受けて pose を更新する。M2 は HMD のみ。
 *
 * @ingroup drv_playspectra
 */

#pragma once

#include "xrt/xrt_compiler.h"
#include "xrt/xrt_defines.h"

#ifdef __cplusplus
extern "C" {
#endif

struct xrt_device;
struct playspectra_state;

/*!
 * @defgroup drv_playspectra PlaySpectra virtual device driver
 * @ingroup drv
 *
 * @brief Externally-controlled virtual XR devices for headless automation/testing.
 */

/*!
 * 既定の制御チャネル TCP ポート。layer=52700 / SteamVR driver=52701 に続く Monado Adapter=52702。
 * 環境変数 PLAYSPECTRA_MONADO_PORT で上書き可能。
 */
#define PLAYSPECTRA_DEFAULT_PORT 52702

/*!
 * 仮想 HMD を生成する。@p center は STAGE 基準の初期 pose(床 y=0)。@p state は共有
 * VirtualDeviceState(HMD はここから pose を読む)。デバイスは state を1つ ref する。
 *
 * @ingroup drv_playspectra
 */
struct xrt_device *
playspectra_hmd_create(const struct xrt_pose *center, struct playspectra_state *state);

/*!
 * NDJSON 制御チャネル(spec §5)。opaque。start で accept ループのスレッドを立てる。
 *
 * @ingroup drv_playspectra
 */
struct playspectra_control;

/*!
 * 制御チャネルを開始する。set_state は @p state に書き込む(デバイスがそこから読む)。
 * @p port が 0 なら PLAYSPECTRA_DEFAULT_PORT(環境変数で上書き可)。制御チャネルは state を
 * 1つ ref する。失敗時は NULL(デバイスは初期 pose のまま動く)。
 *
 * @ingroup drv_playspectra
 */
struct playspectra_control *
playspectra_control_start(struct playspectra_state *state, uint16_t port);

/*!
 * 制御チャネルを停止・解放する。NULL 安全。
 *
 * @ingroup drv_playspectra
 */
void
playspectra_control_stop(struct playspectra_control *ctl);

/*!
 * @dir drivers/playspectra
 *
 * @brief @ref drv_playspectra files.
 */

#ifdef __cplusplus
}
#endif
