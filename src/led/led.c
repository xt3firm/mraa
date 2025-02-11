/*
 * Author: Manivannan Sadhasivam <manivannan.sadhasivam@linaro.org>
 * Copyright (c) 2017 Linaro Ltd.
 *
 * SPDX-License-Identifier: MIT
 */

#include "led.h"
#include "mraa_internal.h"

#include <dirent.h>
#include <errno.h>
#include <fcntl.h>
#include <inttypes.h>
#include <stdlib.h>
#include <string.h>
#include <sys/errno.h>
#include <sys/stat.h>
#include <unistd.h>

#define SYSFS_CLASS_LED "/sys/class/leds"
#define MAX_SIZE 64

static mraa_result_t
mraa_led_get_trigfd(mraa_led_context dev)
{
    char buf[MAX_SIZE];

    snprintf(buf, MAX_SIZE, "%s/%s", dev->led_path, "trigger");
    dev->trig_fd = open(buf, O_RDWR);
    if (dev->trig_fd == -1) {
        syslog(LOG_ERR, "led: trigger: Failed to open 'trigger': %s", strerror(errno));
        return MRAA_ERROR_INVALID_RESOURCE;
    }

    return MRAA_SUCCESS;
}

static mraa_result_t
mraa_led_get_brightfd(mraa_led_context dev)
{
    char buf[MAX_SIZE];

    snprintf(buf, MAX_SIZE, "%s/%s", dev->led_path, "brightness");
    dev->bright_fd = open(buf, O_RDWR);
    if (dev->bright_fd == -1) {
        syslog(LOG_ERR, "led: brightness: Failed to open 'brightness': %s", strerror(errno));
        return MRAA_ERROR_INVALID_RESOURCE;
    }

    return MRAA_SUCCESS;
}

static mraa_result_t
mraa_led_get_maxbrightfd(mraa_led_context dev)
{
    char buf[MAX_SIZE];

    snprintf(buf, MAX_SIZE, "%s/%s", dev->led_path, "max_brightness");
    dev->max_bright_fd = open(buf, O_RDONLY);
    if (dev->max_bright_fd == -1) {
        syslog(LOG_ERR, "led: max_brightness: Failed to open 'max_brightness': %s", strerror(errno));
        return MRAA_ERROR_INVALID_RESOURCE;
    }

    return MRAA_SUCCESS;
}

static mraa_led_context
mraa_led_init_internal(const char* led)
{
    char brightness_path[MAX_SIZE];
    const char *led_name = NULL;
    char *led_path;
    size_t led_path_len;
    DIR* dir;
    struct dirent* entry;
    int cnt = 0;

    mraa_led_context dev = (mraa_led_context) calloc(1, sizeof(struct _led));
    if (dev == NULL) {
        syslog(LOG_CRIT, "led: init: Failed to allocate memory for context");
        return NULL;
    }

    dev->trig_fd = -1;
    dev->bright_fd = -1;
    dev->max_bright_fd = -1;

    if ((dir = opendir(SYSFS_CLASS_LED)) != NULL) {
        /* get the led name from sysfs path */
        while ((entry = readdir(dir)) != NULL) {
            if (strstr((const char*) entry->d_name, led)) {
                led_name = entry->d_name;
                break;
            }
            cnt++;
        }
    }
    dev->count = cnt;
    if (led_name == NULL) {
        syslog(LOG_CRIT, "led: init: unknown device specified");
        if (dir != NULL) {
            closedir(dir);
        }
        free(dev);
        return NULL;
    }

    led_path_len = strlen(SYSFS_CLASS_LED) + strlen(led_name) + 3;
    led_path = calloc(led_path_len, sizeof(char));
    if (led_path == NULL) {
        syslog(LOG_CRIT, "led: init: Failed to allocate memory for LED path");
        closedir(dir);
        return NULL;
    }
    snprintf(led_path, led_path_len, "%s/%s", SYSFS_CLASS_LED, led_name);
    dev->led_path = led_path;

    snprintf(brightness_path, sizeof(brightness_path), "%s/%s", led_path, "brightness");
    if (access(brightness_path, R_OK | W_OK) != 0) {
        syslog(LOG_NOTICE, "led: init: current user doesn't have access rights for using LED %s", led_name);
    }

    closedir(dir);
    return dev;
}

mraa_led_context
mraa_led_init(int index)
{
    if (plat == NULL) {
        syslog(LOG_ERR, "led: init: platform not initialised");
        return NULL;
    }

    if (plat->led_dev_count == 0) {
        syslog(LOG_ERR, "led: init: no led device defined in platform");
        return NULL;
    }

    if (index < 0) {
        syslog(LOG_ERR, "led: init: led index cannot be negative");
        return NULL;
    }

    if (index >= plat->led_dev_count) {
        syslog(LOG_ERR, "led: init: requested led above led device count");
        return NULL;
    }

    return mraa_led_init_internal((char*) plat->led_dev[index].name);
}

mraa_led_context
mraa_led_init_raw(const char* led)
{
    if (plat == NULL) {
        syslog(LOG_ERR, "led: init: platform not initialised");
        return NULL;
    }

    if (led == NULL) {
        syslog(LOG_ERR, "led: init: invalid device specified");
        return NULL;
    }

    return mraa_led_init_internal(led);
}

mraa_result_t
mraa_led_set_brightness(mraa_led_context dev, int value)
{
    char buf[MAX_SIZE];
    int length;

    if (dev == NULL) {
        syslog(LOG_ERR, "led: set_brightness: context is invalid");
        return MRAA_ERROR_INVALID_HANDLE;
    }

    if (dev->trig_fd != -1) {
        close(dev->trig_fd);
        dev->trig_fd = -1;
    }

    if (dev->max_bright_fd != -1) {
        close(dev->max_bright_fd);
        dev->max_bright_fd = -1;
    }

    if (dev->bright_fd == -1) {
        if (mraa_led_get_brightfd(dev) != MRAA_SUCCESS) {
            return MRAA_ERROR_INVALID_RESOURCE;
        }
    }

    if (lseek(dev->bright_fd, 0, SEEK_SET) == -1) {
        syslog(LOG_ERR, "led: set_brightness: Failed to lseek 'brightness': %s", strerror(errno));
        return MRAA_ERROR_UNSPECIFIED;
    }

    length = snprintf(buf, sizeof(buf), "%d", value);
    if (write(dev->bright_fd, buf, length * sizeof(char)) == -1) {
        syslog(LOG_ERR, "led: set_brightness: Failed to write 'brightness': %s", strerror(errno));
        return MRAA_ERROR_UNSPECIFIED;
    }

    return MRAA_SUCCESS;
}

int
mraa_led_read_brightness(mraa_led_context dev)
{
    char buf[3];

    if (dev == NULL) {
        syslog(LOG_ERR, "led: read_brightness: context is invalid");
        return MRAA_ERROR_INVALID_HANDLE;
    }

    if (dev->trig_fd != -1) {
        close(dev->trig_fd);
        dev->trig_fd = -1;
    }

    if (dev->max_bright_fd != -1) {
        close(dev->max_bright_fd);
        dev->max_bright_fd = -1;
    }

    if (dev->bright_fd == -1) {
        if (mraa_led_get_brightfd(dev) != MRAA_SUCCESS) {
            return MRAA_ERROR_INVALID_RESOURCE;
        }
    } else {
        lseek(dev->bright_fd, 0, SEEK_SET);
    }

    if (read(dev->bright_fd, buf, 3 * sizeof(char)) == -1) {
        syslog(LOG_ERR, "led: read_brightness: Failed to read 'brightness': %s", strerror(errno));
        return MRAA_ERROR_UNSPECIFIED;
    }
    lseek(dev->bright_fd, 0, SEEK_SET);

    return (int) atoi(buf);
}

int
mraa_led_read_max_brightness(mraa_led_context dev)
{
    char buf[8];

    if (dev == NULL) {
        syslog(LOG_ERR, "led: read_max_brightness: context is invalid");
        return MRAA_ERROR_INVALID_HANDLE;
    }

    if (dev->trig_fd != -1) {
        close(dev->trig_fd);
        dev->trig_fd = -1;
    }

    if (dev->bright_fd != -1) {
        close(dev->bright_fd);
        dev->bright_fd = -1;
    }

    if (dev->max_bright_fd == -1) {
        if (mraa_led_get_maxbrightfd(dev) != MRAA_SUCCESS) {
            return MRAA_ERROR_INVALID_RESOURCE;
        }
    } else {
        lseek(dev->max_bright_fd, 0, SEEK_SET);
    }

    ssize_t l = read(dev->max_bright_fd, buf, sizeof(buf));
    if (l == -1) {
        syslog(LOG_ERR, "led: read_max_brightness: Failed to read 'max_brightness': %s", strerror(errno));
        return MRAA_ERROR_UNSPECIFIED;
    }
    if (l >= sizeof(buf))
        l = sizeof(buf) - 1;
    buf[l] = '\0';

    lseek(dev->max_bright_fd, 0, SEEK_SET);

    return (int) atoi(buf);
}

mraa_result_t
mraa_led_set_trigger(mraa_led_context dev, const char* trigger)
{
    char buf[MAX_SIZE];
    int length;

    if (dev == NULL) {
        syslog(LOG_ERR, "led: set_trigger: context is invalid");
        return MRAA_ERROR_INVALID_HANDLE;
    }

    if (dev->bright_fd != -1) {
        close(dev->bright_fd);
        dev->bright_fd = -1;
    }

    if (dev->max_bright_fd != -1) {
        close(dev->max_bright_fd);
        dev->max_bright_fd = -1;
    }

    if (trigger == NULL) {
        syslog(LOG_ERR, "led: trigger: invalid trigger specified");
        return MRAA_ERROR_INVALID_RESOURCE;
    }

    if (dev->trig_fd == -1) {
        if (mraa_led_get_trigfd(dev) != MRAA_SUCCESS) {
            return MRAA_ERROR_INVALID_RESOURCE;
        }
    }

    if (lseek(dev->trig_fd, 0, SEEK_SET) == -1) {
        syslog(LOG_ERR, "led: set_trigger: Failed to lseek 'trigger': %s", strerror(errno));
        return MRAA_ERROR_UNSPECIFIED;
    }

    length = snprintf(buf, sizeof(buf), "%s", trigger);
    if (write(dev->trig_fd, buf, length * sizeof(char)) == -1) {
        syslog(LOG_ERR, "led: set_trigger: Failed to write 'trigger': %s", strerror(errno));
        return MRAA_ERROR_UNSPECIFIED;
    }

    return MRAA_SUCCESS;
}

mraa_result_t
mraa_led_clear_trigger(mraa_led_context dev)
{
    char buf[1] = { '0' };

    if (dev == NULL) {
        syslog(LOG_ERR, "led: clear_trigger: context is invalid");
        return MRAA_ERROR_INVALID_HANDLE;
    }

    if (dev->trig_fd != -1) {
        close(dev->trig_fd);
        dev->trig_fd = -1;
    }

    if (dev->max_bright_fd != -1) {
        close(dev->max_bright_fd);
        dev->max_bright_fd = -1;
    }

    if (dev->bright_fd == -1) {
        if (mraa_led_get_brightfd(dev) != MRAA_SUCCESS) {
            return MRAA_ERROR_INVALID_RESOURCE;
        }
    }

    if (lseek(dev->bright_fd, 0, SEEK_SET) == -1) {
        syslog(LOG_ERR, "led: clear_trigger: Failed to lseek 'brightness': %s", strerror(errno));
        return MRAA_ERROR_UNSPECIFIED;
    }

    /* writing 0 to brightness clears trigger */
    if (write(dev->bright_fd, buf, 1) == -1) {
        syslog(LOG_ERR, "led: clear_trigger: Failed to write 'brightness': %s", strerror(errno));
        return MRAA_ERROR_UNSPECIFIED;
    }

    return MRAA_SUCCESS;
}

mraa_result_t
mraa_led_close(mraa_led_context dev)
{
    if (dev == NULL) {
        syslog(LOG_ERR, "led: close: context is invalid");
        return MRAA_ERROR_INVALID_HANDLE;
    }

    if (dev->bright_fd != -1) {
        close(dev->bright_fd);
    }

    if (dev->trig_fd != -1) {
        close(dev->trig_fd);
    }

    if (dev->max_bright_fd != -1) {
        close(dev->max_bright_fd);
    }

    free((void *)dev->led_path);
    free(dev);

    return MRAA_SUCCESS;
}
