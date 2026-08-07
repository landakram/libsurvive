#include "../ootx_decoder.h"
#include "../survive_default_devices.h"
#include "../survive_internal.h"
#include "../survive_kalman_lighthouses.h"
#include "../survive_kalman_tracker.h"
#include "test_case.h"
#include <poser.h>
#include <string.h>
#include <survive_api.h>

static SurviveContext *create_lighthouse_test_context(bool disable_calibrate, bool lock_known_lighthouses) {
	char *argv[] = {"test-lighthouse-lock", "--simulator", "--no-threaded-posers", "--configfile", "/dev/null",
					"--simulator-lh-gen", "2", "--disable-calibrate", "--lock-known-lighthouses"};
	int argc = 7;
	if (disable_calibrate) {
		argv[argc++] = "--disable-calibrate";
	}
	if (lock_known_lighthouses) {
		argv[argc++] = "--lock-known-lighthouses";
	}

	SurviveContext *ctx = survive_init_internal(argc, argv, 0, 0);
	if (ctx == 0) {
		return 0;
	}

	ctx->activeLighthouses = 1;
	ctx->floor_offset = .75;
	ctx->bsd[0].PositionSet = true;
	ctx->bsd[0].OOTXSet = true;
	ctx->bsd[0].BaseStationID = 0x12345678;
	ctx->bsd[0].mode = 1;
	ctx->bsd[0].Pose = (SurvivePose){.Pos = {1, 2, 3}, .Rot = {1, 0, 0, 0}};
	ctx->bsd[0].accel[2] = 1;
	ctx->bsd_map[1] = 0;

	if (survive_startup(ctx) != SURVIVE_OK) {
		return 0;
	}
	return ctx;
}

static void fill_gen2_ootx_packet(uint8_t data[43], uint32_t lighthouse_id, int8_t accel_x, uint8_t unlock_count) {
	memset(data, 0, 43);
	memcpy(data + 2, &lighthouse_id, sizeof(lighthouse_id));
	data[7] = 0x3c;
	data[14] = unlock_count;
	data[20] = (uint8_t)accel_x;
	data[22] = 1;
	data[31] = 1;
}

static ootx_decoder_context *get_ootx_decoder(SurviveContext *ctx) {
	if (ctx == 0 || ctx->objs_ct == 0) {
		return 0;
	}
	ctx->bsd[0].OOTXChecked = false;
	survive_default_sync_process(ctx->objs[0], 1, 1, false, true);
	return ctx->bsd[0].ootx_data;
}

static void *attempt_concurrent_frozen_mutations(void *user) {
	SurviveContext *ctx = user;
	for (int i = 0; i < 16; i++) {
		survive_get_ctx_lock(ctx);
		SurvivePose changed = ctx->bsd[0].Pose;
		changed.Pos[0] += .25;
		survive_default_raw_lighthouse_pose_process(ctx, 0, &changed);
		survive_reset_lighthouse_position(ctx, 0);
		survive_kalman_lighthouse_integrate_observation(ctx->bsd[0].tracker, &changed, 0);
		survive_release_ctx_lock(ctx);
	}
	return 0;
}

TEST(Survive, FrozenLighthouseRejectsPoseAndKalmanMutations) {
	SurviveContext *ctx = create_lighthouse_test_context(true, true);
	if (ctx == 0) {
		return survive_test_assert();
	}

	SurvivePose expected = ctx->bsd[0].Pose;
	SurvivePose changed = expected;
	changed.Pos[0] += .25;
	survive_default_raw_lighthouse_pose_process(ctx, 0, &changed);
	ASSERT_DOUBLE_ARRAY_EQ(7, ((FLT *)&ctx->bsd[0].Pose), ((FLT *)&expected));

	changed = expected;
	changed.Rot[0] = .7071067811865476;
	changed.Rot[1] = .7071067811865476;
	survive_default_raw_lighthouse_pose_process(ctx, 0, &changed);
	ASSERT_DOUBLE_ARRAY_EQ(7, ((FLT *)&ctx->bsd[0].Pose), ((FLT *)&expected));
	survive_kalman_lighthouse_update_position(ctx->bsd[0].tracker, &changed);
	ASSERT_DOUBLE_ARRAY_EQ(7, ((FLT *)&ctx->bsd[0].tracker->state.Lighthouse), ((FLT *)&expected));
	survive_kalman_lighthouse_integrate_observation(ctx->bsd[0].tracker, &changed, 0);
	ASSERT_DOUBLE_ARRAY_EQ(7, ((FLT *)&ctx->bsd[0].tracker->state.Lighthouse), ((FLT *)&expected));
	ASSERT_DOUBLE_ARRAY_EQ(7, ((FLT *)&ctx->bsd[0].Pose), ((FLT *)&expected));

	survive_default_raw_lighthouse_pose_process(ctx, 0, 0);
	survive_reset_lighthouse_position(ctx, 0);
	ASSERT_EQ(ctx->bsd[0].PositionSet, 1);
	ASSERT_DOUBLE_ARRAY_EQ(7, ((FLT *)&ctx->bsd[0].Pose), ((FLT *)&expected));
	survive_reset_lighthouse_positions(ctx);
	ASSERT_EQ(ctx->bsd[0].PositionSet, 1);
	ASSERT_DOUBLE_EQ(ctx->floor_offset, .75);

	survive_close(ctx);
	return 0;
}

TEST(Survive, FrozenLighthouseRetainsGeometryWhenTrackerLosesTracking) {
	SurviveContext *ctx = create_lighthouse_test_context(true, true);
	if (ctx == 0 || ctx->objs_ct == 0) {
		return survive_test_assert();
	}

	SurviveObject *object = ctx->objs[0];
	object->OutPose = (SurvivePose){.Pos = {4, 5, 6}, .Rot = {1, 0, 0, 0}};
	object->OutPoseIMU = object->OutPose;
	SurvivePose expected = ctx->bsd[0].Pose;
	survive_kalman_tracker_lost_tracking(object->tracker, true);
	ASSERT_EQ(quatiszero(object->OutPose.Rot), true);
	ASSERT_EQ(quatiszero(object->OutPoseIMU.Rot), true);
	ASSERT_EQ(ctx->bsd[0].PositionSet, 1);
	ASSERT_DOUBLE_EQ(ctx->floor_offset, .75);
	ASSERT_DOUBLE_ARRAY_EQ(7, ((FLT *)&ctx->bsd[0].Pose), ((FLT *)&expected));

	survive_close(ctx);
	return 0;
}

TEST(Survive, FrozenLighthouseRetainsGeometryAcrossConcurrentUpdates) {
	SurviveContext *ctx = create_lighthouse_test_context(true, true);
	if (ctx == 0) {
		return survive_test_assert();
	}

	SurvivePose expected = ctx->bsd[0].Pose;
	og_thread_t threads[4];
	for (int i = 0; i < 4; i++) {
		threads[i] = OGCreateThread(attempt_concurrent_frozen_mutations, "frozen lighthouse test", ctx);
	}
	for (int i = 0; i < 4; i++) {
		OGJoinThread(threads[i]);
	}
	ASSERT_EQ(ctx->bsd[0].PositionSet, 1);
	ASSERT_DOUBLE_ARRAY_EQ(7, ((FLT *)&ctx->bsd[0].Pose), ((FLT *)&expected));
	ASSERT_DOUBLE_ARRAY_EQ(7, ((FLT *)&ctx->bsd[0].tracker->state.Lighthouse), ((FLT *)&expected));

	survive_close(ctx);
	return 0;
}

TEST(Survive, FrozenLighthouseRetainsGeometryAndOotxMetadata) {
	SurviveContext *ctx = create_lighthouse_test_context(true, true);
	ootx_decoder_context *decoder = get_ootx_decoder(ctx);
	if (decoder == 0 || decoder->ootx_packet_clbk == 0) {
		return survive_test_assert();
	}

	uint8_t data[43];
	fill_gen2_ootx_packet(data, ctx->bsd[0].BaseStationID, 0, 4);
	ootx_packet packet = {.length = sizeof(data), .data = data};
	ctx->bsd[0].OOTXSet = false;
	decoder->ootx_packet_clbk(decoder, &packet);
	ASSERT_EQ(ctx->bsd[0].PositionSet, 1);
	ASSERT_EQ(ctx->bsd[0].OOTXSet, 1);
	ASSERT_DOUBLE_EQ(ctx->bsd[0].fcal[0].phase, 1.);
	ASSERT_EQ(ctx->bsd[0].sys_unlock_count, 4);

	fill_gen2_ootx_packet(data, ctx->bsd[0].BaseStationID, 1, 7);
	decoder->ootx_packet_clbk(decoder, &packet);
	ASSERT_EQ(ctx->bsd[0].PositionSet, 1);
	ASSERT_DOUBLE_EQ(ctx->bsd[0].accel[0], 0.);
	ASSERT_DOUBLE_EQ(ctx->bsd[0].accel[2], 1.);
	ASSERT_DOUBLE_EQ(ctx->bsd[0].fcal[0].phase, 1.);
	ASSERT_EQ(ctx->bsd[0].sys_unlock_count, 7);

	survive_close(ctx);
	return 0;
}

TEST(Survive, FrozenGen1LighthouseRetainsGeometryAndOotxMetadata) {
	SurviveContext *ctx = create_lighthouse_test_context(true, true);
	if (ctx == 0 || ctx->objs_ct == 0) {
		return survive_test_assert();
	}

	ctx->lh_version = 0;
	ctx->bsd[0].OOTXChecked = false;
	survive_default_sync_process(ctx->objs[0], 0, 1, false, false);
	ootx_decoder_context *decoder = ctx->bsd[0].ootx_data;
	if (decoder == 0 || decoder->ootx_packet_clbk == 0) {
		return survive_test_assert();
	}

	uint8_t data[43];
	fill_gen2_ootx_packet(data, ctx->bsd[0].BaseStationID, 1, 11);
	ootx_packet packet = {.length = sizeof(data), .data = data};
	decoder->ootx_packet_clbk(decoder, &packet);
	ASSERT_EQ(ctx->bsd[0].PositionSet, 1);
	ASSERT_DOUBLE_EQ(ctx->bsd[0].accel[0], 0.);
	ASSERT_DOUBLE_EQ(ctx->bsd[0].accel[2], 1.);
	ASSERT_EQ(ctx->bsd[0].sys_unlock_count, 11);

	survive_close(ctx);
	return 0;
}

TEST(Survive, FrozenLighthouseRejectsUnknownChannelsAndIdentityRemaps) {
	SurviveContext *ctx = create_lighthouse_test_context(true, true);
	ootx_decoder_context *decoder = get_ootx_decoder(ctx);
	if (decoder == 0 || decoder->ootx_packet_clbk == 0) {
		return survive_test_assert();
	}

	ASSERT_EQ(survive_get_bsd_idx(ctx, 2), SURVIVE_BSD_IDX_IGNORED);
	uint8_t data[43];
	fill_gen2_ootx_packet(data, 0x87654321, 0, 1);
	ootx_packet packet = {.length = sizeof(data), .data = data};
	decoder->ootx_packet_clbk(decoder, &packet);
	ASSERT_EQ(ctx->bsd[0].BaseStationID, 0x12345678);
	ASSERT_EQ(ctx->bsd[0].PositionSet, 1);

	survive_close(ctx);
	return 0;
}

TEST(Survive, FrozenLighthousePreservesLowConfidenceWithoutRecalibration) {
	SurviveContext *ctx = create_lighthouse_test_context(true, true);
	if (ctx == 0) {
		return survive_test_assert();
	}

	ctx->bsd[0].confidence = .5;
	ASSERT_DOUBLE_EQ(survive_lighthouse_adjust_confidence(ctx, 0, -1), -.5);
	ASSERT_EQ(ctx->bsd[0].PositionSet, 1);

	survive_close(ctx);
	return 0;
}

TEST(Survive, NormalLighthouseCalibrationAndDiscoveryStillWork) {
	SurviveContext *ctx = create_lighthouse_test_context(false, false);
	if (ctx == 0) {
		return survive_test_assert();
	}

	SurvivePose changed = ctx->bsd[0].Pose;
	changed.Pos[0] += .25;
	survive_default_raw_lighthouse_pose_process(ctx, 0, &changed);
	ASSERT_DOUBLE_EQ(ctx->bsd[0].Pose.Pos[0], changed.Pos[0]);
	ASSERT_EQ(survive_get_bsd_idx(ctx, 2), 1);
	survive_reset_lighthouse_position(ctx, 0);
	ASSERT_EQ(ctx->bsd[0].PositionSet, 0);
	survive_reset_lighthouse_positions(ctx);
	ASSERT_DOUBLE_EQ(ctx->floor_offset, 0.);

	survive_close(ctx);
	return 0;
}

TEST(Survive, NormalLighthouseOotxStillRecalibratesChangedAccelerometer) {
	SurviveContext *ctx = create_lighthouse_test_context(false, false);
	ootx_decoder_context *decoder = get_ootx_decoder(ctx);
	if (decoder == 0 || decoder->ootx_packet_clbk == 0) {
		return survive_test_assert();
	}

	uint8_t data[43];
	fill_gen2_ootx_packet(data, ctx->bsd[0].BaseStationID, 1, 12);
	ootx_packet packet = {.length = sizeof(data), .data = data};
	decoder->ootx_packet_clbk(decoder, &packet);
	ASSERT_EQ(ctx->bsd[0].PositionSet, 0);
	ASSERT_DOUBLE_EQ(ctx->bsd[0].accel[0], 1.);
	ASSERT_EQ(ctx->bsd[0].sys_unlock_count, 12);

	survive_close(ctx);
	return 0;
}

TEST(Survive, LighthouseCanBeRecalibratedAndFrozenAgain) {
	SurviveContext *ctx = create_lighthouse_test_context(true, true);
	if (ctx == 0) {
		return survive_test_assert();
	}

	survive_configi(ctx, "disable-calibrate", SC_OVERRIDE | SC_SET, false);
	survive_configi(ctx, "lock-known-lighthouses", SC_OVERRIDE | SC_SET, false);
	survive_reset_lighthouse_position(ctx, 0);
	ASSERT_EQ(ctx->bsd[0].PositionSet, 0);
	SurvivePose recalibrated = {.Pos = {4, 5, 6}, .Rot = {1, 0, 0, 0}};
	survive_default_raw_lighthouse_pose_process(ctx, 0, &recalibrated);
	ASSERT_EQ(ctx->bsd[0].PositionSet, 1);
	ASSERT_DOUBLE_ARRAY_EQ(7, ((FLT *)&ctx->bsd[0].Pose), ((FLT *)&recalibrated));

	survive_configi(ctx, "disable-calibrate", SC_OVERRIDE | SC_SET, true);
	survive_configi(ctx, "lock-known-lighthouses", SC_OVERRIDE | SC_SET, true);
	SurvivePose changed = recalibrated;
	changed.Pos[0] += .25;
	survive_default_raw_lighthouse_pose_process(ctx, 0, &changed);
	ASSERT_DOUBLE_ARRAY_EQ(7, ((FLT *)&ctx->bsd[0].Pose), ((FLT *)&recalibrated));

	survive_close(ctx);
	return 0;
}

TEST(Survive, DisabledCalibrationWithoutIdentityLockIgnoresAccelerometerNoise) {
	SurviveContext *ctx = create_lighthouse_test_context(true, false);
	ootx_decoder_context *decoder = get_ootx_decoder(ctx);
	if (decoder == 0 || decoder->ootx_packet_clbk == 0) {
		return survive_test_assert();
	}

	uint8_t data[43];
	fill_gen2_ootx_packet(data, ctx->bsd[0].BaseStationID, 1, 9);
	ootx_packet packet = {.length = sizeof(data), .data = data};
	decoder->ootx_packet_clbk(decoder, &packet);
	ASSERT_EQ(ctx->bsd[0].PositionSet, 1);
	ASSERT_DOUBLE_EQ(ctx->bsd[0].accel[0], 0.);
	ASSERT_EQ(ctx->bsd[0].sys_unlock_count, 9);
	survive_reset_lighthouse_position(ctx, 0);
	ASSERT_EQ(ctx->bsd[0].PositionSet, 0);

	survive_close(ctx);
	return 0;
}

TEST(Survive, FrozenLighthouseGenerationMismatchFailsClosed) {
	SurviveContext *ctx = create_lighthouse_test_context(true, true);
	if (ctx == 0 || ctx->objs_ct == 0) {
		return survive_test_assert();
	}

	SurvivePose expected = ctx->bsd[0].Pose;
	ctx->lh_version = -1;
	ctx->lh_version_configed = 1;
	survive_default_gen_detected_process(ctx->objs[0], 0);
	ASSERT_EQ(ctx->currentError, SURVIVE_ERROR_INVALID_CONFIG);
	ASSERT_EQ(ctx->bsd[0].PositionSet, 1);
	ASSERT_DOUBLE_ARRAY_EQ(7, ((FLT *)&ctx->bsd[0].Pose), ((FLT *)&expected));

	survive_close(ctx);
	return 0;
}

TEST(Survive, FrozenLighthouseRejectsSteamVrCalibration) {
	SurviveContext *ctx = create_lighthouse_test_context(true, true);
	if (ctx == 0) {
		return survive_test_assert();
	}

	SurvivePose expected = ctx->bsd[0].Pose;
	char calibration[] = "{}";
	ASSERT_EQ(survive_load_steamvr_lighthousedb(ctx, calibration, sizeof(calibration) - 1), -1);
	ASSERT_EQ(ctx->currentError, SURVIVE_ERROR_INVALID_CONFIG);
	ASSERT_EQ(ctx->bsd[0].PositionSet, 1);
	ASSERT_DOUBLE_ARRAY_EQ(7, ((FLT *)&ctx->bsd[0].Pose), ((FLT *)&expected));

	survive_close(ctx);
	return 0;
}

TEST(Survive, FrozenLighthouseWithoutConfiguredGeometryFailsClosed) {
	char *argv[] = {"test-lighthouse-lock", "--configfile", "/dev/null", "--disable-calibrate",
					"--lock-known-lighthouses"};
	SurviveContext *ctx = survive_init_internal(sizeof(argv) / sizeof(argv[0]), argv, 0, 0);
	if (ctx == 0) {
		return survive_test_assert();
	}

	ASSERT_EQ(survive_startup(ctx), SURVIVE_ERROR_INVALID_CONFIG);
	ASSERT_EQ(ctx->currentError, SURVIVE_ERROR_INVALID_CONFIG);
	survive_close(ctx);
	return 0;
}

TEST(Survive, FrozenSimpleApiWithoutConfiguredGeometryReturnsNull) {
	char *argv[] = {"test-lighthouse-lock", "--configfile", "/dev/null", "--disable-calibrate",
					"--lock-known-lighthouses"};
	ASSERT_EQ(survive_simple_init(sizeof(argv) / sizeof(argv[0]), argv), 0);
	return 0;
}
