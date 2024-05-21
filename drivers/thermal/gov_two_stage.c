// SPDX-License-Identifier: GPL-2.0-only
/*
 *  two_stage.c - A two-stage thermal governor (off-on)
 *
 *  Copyright (C) 2015 Russell Robinson <rrobinson@phytec.com>
 *
 *  Based on drivers/thermal/gov_step_wise.c - A step-by-step Thermal throttling governor
 *
 *  Copyright (C) 2012 Intel Corp
 *  Copyright (C) 2012 Durgadoss R <durgadoss.r@intel.com>
 */

#include <linux/thermal.h>

#include "thermal_core.h"

/*
 * Temperature > trip_point: turn cooling device on
 * Temperature < trip_point: turn cooling device off
 */
static unsigned long get_target_state(struct thermal_instance *instance,
				bool throttle)
{
	struct thermal_cooling_device *cdev = instance->cdev;
	unsigned long cur_state;
	unsigned long next_target;

	/*
	 * We keep this instance the way it is by default.
	 * Otherwise, we use the current state of the
	 * cdev in use to determine the next_target.
	 */
	cdev->ops->get_cur_state(cdev, &cur_state);
	next_target = instance->target;
	dev_dbg(&cdev->device, "cur_state=%ld\n", cur_state);

	if (throttle) {
		next_target = (cur_state == 1) ?
				cur_state : cur_state + 1;
	} else {
		next_target = (cur_state == 0) ?
				cur_state : cur_state - 1;
	}

	return next_target;
}

static void update_passive_instance(struct thermal_zone_device *tz,
				enum thermal_trip_type type, int value)
{
	/*
	 * If value is +1, activate a passive instance.
	 * If value is -1, deactivate a passive instance.
	 */
	if (type == THERMAL_TRIP_PASSIVE)
		tz->passive += value;
}

static void thermal_zone_trip_update(struct thermal_zone_device *tz, int trip)
{
	int trip_temp;
	enum thermal_trip_type trip_type;
	struct thermal_instance *instance;
	bool throttle = false;
	int old_target;

	tz->ops->get_trip_temp(tz, trip, &trip_temp);
	tz->ops->get_trip_type(tz, trip, &trip_type);

	if (tz->temperature >= trip_temp)
		throttle = true;

	dev_dbg(&tz->device, "Trip%d[type=%d,temp=%d],throttle=%d\n",
				trip, trip_type, trip_temp, throttle);

	list_for_each_entry(instance, &tz->thermal_instances, tz_node) {
		if (instance->trip != trip)
			continue;

		old_target = instance->target;
		instance->target = get_target_state(instance, throttle);
		dev_dbg(&instance->cdev->device, "old_target=%d, target=%d\n",
					old_target, (int)instance->target);

		if (old_target == instance->target)
			continue;

		/* Activate a passive thermal instance */
		if (old_target == THERMAL_NO_TARGET &&
			instance->target != THERMAL_NO_TARGET)
			update_passive_instance(tz, trip_type, 1);
		/* Deactivate a passive thermal instance */
		else if (old_target != THERMAL_NO_TARGET &&
			instance->target == THERMAL_NO_TARGET)
			update_passive_instance(tz, trip_type, -1);


		instance->cdev->updated = false; /* cdev needs update */
	}
}

/**
 * two_stage_throttle - throttles devices asscciated with the given zone
 * @tz - thermal_zone_device
 * @trip - the trip point
 * @trip_type - type of the trip point
 *
 */
static int two_stage_throttle(struct thermal_zone_device *tz, int trip)
{
	struct thermal_instance *instance;

	lockdep_assert_held(&tz->lock);

	thermal_zone_trip_update(tz, trip);

	list_for_each_entry(instance, &tz->thermal_instances, tz_node)
		thermal_cdev_update(instance->cdev);

	return 0;
}

static struct thermal_governor thermal_gov_two_stage = {
	.name		= "two_stage",
	.throttle	= two_stage_throttle,
};

THERMAL_GOVERNOR_DECLARE(thermal_gov_two_stage);
