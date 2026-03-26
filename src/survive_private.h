#pragma once

#include <stdbool.h>
#include <stdint.h>

struct SurviveExternalPose {
	char name[32];
	SurvivePose pose;
};

struct SurviveContext_private {
	og_sema_t poll_sema;
	survive_run_time_fn runTimeFn;
	void *runTimeFnUser;
	double lastRunTime;

	double callbackStatsTimeBetween;
	double lastCallbackStats;

	struct SurviveExternalPose ExternalPoses[16];
	SurvivePose external2world;
	uint16_t allowed_lighthouse_channels_mask;
	uint16_t warned_disallowed_channels_mask;
	bool allowed_lighthouse_channels_configured;
};
