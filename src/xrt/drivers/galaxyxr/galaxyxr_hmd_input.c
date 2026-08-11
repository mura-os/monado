// Copyright 2026, Stanislav Aleksandrov
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief  Samsung Galaxy XR headset inputs.
 * @author Stanislav Aleksandrov <lightofmysoul@bringo.com>
 * @ingroup drv_galaxyxr
 */

#include "galaxyxr_hmd_input.h"

#include "os/os_time.h"

#include "util/u_debug.h"

#include <errno.h>
#include <fcntl.h>
#include <linux/input.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define GXR_POWER_DEVICE_NAME "pmic_pwrkey"
#define GXR_BOOLEAN_STATE_BIT INT64_MIN
#define GXR_BOOLEAN_TIMESTAMP_MASK INT64_MAX

enum galaxyxr_input_index
{
	GXR_INPUT_HEAD_POSE,
	GXR_INPUT_HEAD_DETECT,
	GXR_INPUT_POWER_CLICK,
#ifdef XRT_BUILD_DRIVER_GALAXYXR_EYE_TRACKING
	GXR_INPUT_EYE_GAZE_POSE,
#endif
};

DEBUG_GET_ONCE_OPTION(galaxyxr_power_device, "GALAXYXR_POWER_DEVICE", "/dev/input/event2")

#define GXR_DEBUG(i, ...) U_LOG_XDEV_IFL_D((i)->xdev, *(i)->log_level, __VA_ARGS__)
#define GXR_INFO(i, ...) U_LOG_XDEV_IFL_I((i)->xdev, *(i)->log_level, __VA_ARGS__)
#define GXR_WARN(i, ...) U_LOG_XDEV_IFL_W((i)->xdev, *(i)->log_level, __VA_ARGS__)

static struct xrt_binding_input_pair vive_pro_inputs_galaxyxr[] = {
    {XRT_INPUT_VIVEPRO_SYSTEM_CLICK, XRT_INPUT_GALAXYXR_POWER_CLICK},
};

#ifdef XRT_BUILD_DRIVER_GALAXYXR_EYE_TRACKING
static struct xrt_binding_input_pair eye_gaze_inputs_galaxyxr[] = {
    {XRT_INPUT_GENERIC_EYE_GAZE_POSE, XRT_INPUT_GENERIC_EYE_GAZE_POSE},
};
#endif

static struct xrt_binding_profile galaxyxr_binding_profiles[] = {
#ifdef XRT_BUILD_DRIVER_GALAXYXR_EYE_TRACKING
    {
        .name = XRT_DEVICE_EYE_GAZE_INTERACTION,
        .inputs = eye_gaze_inputs_galaxyxr,
        .input_count = ARRAY_SIZE(eye_gaze_inputs_galaxyxr),
    },
#endif
    {
        .name = XRT_DEVICE_VIVE_PRO,
        .inputs = vive_pro_inputs_galaxyxr,
        .input_count = ARRAY_SIZE(vive_pro_inputs_galaxyxr),
    },
};

static void
boolean_state_store(xrt_atomic_s64_t *state, bool value)
{
	int64_t packed = os_monotonic_get_ns();
	packed |= value ? GXR_BOOLEAN_STATE_BIT : 0;
	xrt_atomic_s64_store(state, packed);
}

static bool
boolean_state_load(xrt_atomic_s64_t *state, int64_t *out_timestamp_ns)
{
	int64_t packed = xrt_atomic_s64_load(state);
	if (out_timestamp_ns != NULL) {
		*out_timestamp_ns = packed & GXR_BOOLEAN_TIMESTAMP_MASK;
	}
	return (packed & GXR_BOOLEAN_STATE_BIT) != 0;
}

static bool
evdev_bit_is_set(const uint8_t *bits, uint32_t bit)
{
	return (bits[bit / 8] & (uint8_t)(1u << (bit % 8))) != 0;
}

void
galaxyxr_hmd_input_init(struct galaxyxr_hmd_input *input,
                        struct xrt_device *xdev,
                        enum u_logging_level *log_level)
{
	input->xdev = xdev;
	input->log_level = log_level;

	xdev->inputs[GXR_INPUT_HEAD_POSE].name = XRT_INPUT_GENERIC_HEAD_POSE;
	xdev->inputs[GXR_INPUT_HEAD_DETECT].name = XRT_INPUT_GENERIC_HEAD_DETECT;
	xdev->inputs[GXR_INPUT_POWER_CLICK].name = XRT_INPUT_GALAXYXR_POWER_CLICK;
	xdev->inputs[GXR_INPUT_POWER_CLICK].active = true;
#ifdef XRT_BUILD_DRIVER_GALAXYXR_EYE_TRACKING
	xdev->inputs[GXR_INPUT_EYE_GAZE_POSE].name = XRT_INPUT_GENERIC_EYE_GAZE_POSE;
#endif
	xdev->binding_profiles = galaxyxr_binding_profiles;
	xdev->binding_profile_count = ARRAY_SIZE(galaxyxr_binding_profiles);

	boolean_state_store(&input->presence_state, true);
	boolean_state_store(&input->power_button_state, false);
	xdev->inputs[GXR_INPUT_HEAD_DETECT].value.boolean = true;
	xdev->supported.presence = true;
	xdev->supported.presence_display_power = true;
}

xrt_result_t
galaxyxr_hmd_input_update(struct galaxyxr_hmd_input *input)
{
	struct xrt_input *head_detect = &input->xdev->inputs[GXR_INPUT_HEAD_DETECT];
	head_detect->value.boolean = boolean_state_load(&input->presence_state, &head_detect->timestamp);

	struct xrt_input *power = &input->xdev->inputs[GXR_INPUT_POWER_CLICK];
	power->value.boolean = boolean_state_load(&input->power_button_state, &power->timestamp);
	return XRT_SUCCESS;
}

void
galaxyxr_hmd_input_handle_proximity(struct galaxyxr_hmd_input *input, float state, float adc)
{
	bool present = state == 1.0f;
	bool was_present = boolean_state_load(&input->presence_state, NULL);
	if (present != was_present) {
		GXR_INFO(input, "Proximity sensor: user is %s (state %.0f, adc %.0f)",
		         present ? "present" : "not present", state, adc);
	}
	boolean_state_store(&input->presence_state, present);
}

int
galaxyxr_hmd_input_open_power_button(struct galaxyxr_hmd_input *input)
{
	const char *path = debug_get_option_galaxyxr_power_device();
	int fd = open(path, O_RDONLY | O_NONBLOCK | O_CLOEXEC);
	if (fd < 0) {
		GXR_WARN(input, "Could not open power button %s: %s (the Monado service needs read access)", path,
		         strerror(errno));
		return -1;
	}

	char name[256] = {0};
	if (ioctl(fd, EVIOCGNAME(sizeof(name)), name) < 0) {
		GXR_WARN(input, "Could not read the input device name from %s: %s", path, strerror(errno));
		goto fail;
	}
	if (strcmp(name, GXR_POWER_DEVICE_NAME) != 0) {
		GXR_WARN(input, "Power button path %s is '%s', expected '%s'", path, name, GXR_POWER_DEVICE_NAME);
		goto fail;
	}

	uint8_t key_bits[(KEY_CNT + 7) / 8] = {0};
	if (ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(key_bits)), key_bits) < 0) {
		GXR_WARN(input, "Could not query the keys supported by %s: %s", path, strerror(errno));
		goto fail;
	}
	if (!evdev_bit_is_set(key_bits, KEY_POWER)) {
		GXR_WARN(input, "Input device %s does not support KEY_POWER", path);
		goto fail;
	}

	// evdev normally broadcasts each event to every open reader. Take exclusive
	// ownership so the desktop does not also handle KEY_POWER while Monado runs.
	// Closing fd releases the grab automatically.
	if (ioctl(fd, EVIOCGRAB, 1) < 0) {
		GXR_WARN(input, "Could not exclusively grab power button %s: %s", path, strerror(errno));
		goto fail;
	}

	bool pressed = false;
	uint8_t key_state[(KEY_CNT + 7) / 8] = {0};
	if (ioctl(fd, EVIOCGKEY(sizeof(key_state)), key_state) >= 0) {
		pressed = evdev_bit_is_set(key_state, KEY_POWER);
	}
	boolean_state_store(&input->power_button_state, pressed);

	GXR_INFO(input, "Power button: %s (%s, KEY_POWER, exclusively grabbed)", name, path);
	return fd;

fail:
	close(fd);
	return -1;
}

bool
galaxyxr_hmd_input_poll_power_button(struct galaxyxr_hmd_input *input, int fd)
{
	struct input_event events[16];
	for (;;) {
		ssize_t ret = read(fd, events, sizeof(events));
		if (ret < 0) {
			if (errno == EINTR) {
				continue;
			}
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				return true;
			}

			GXR_WARN(input, "Power button read failed: %s", strerror(errno));
			break;
		}
		if (ret == 0) {
			GXR_WARN(input, "Power button device was disconnected");
			break;
		}

		size_t event_count = (size_t)ret / sizeof(events[0]);
		for (size_t i = 0; i < event_count; i++) {
			const struct input_event *event = &events[i];
			if (event->type != EV_KEY || event->code != KEY_POWER) {
				continue;
			}

			bool pressed = event->value != 0;
			boolean_state_store(&input->power_button_state, pressed);
			GXR_DEBUG(input, "Power button %s", pressed ? "pressed" : "released");
		}
	}

	return false;
}

void
galaxyxr_hmd_input_reset_power_button(struct galaxyxr_hmd_input *input)
{
	boolean_state_store(&input->power_button_state, false);
}
