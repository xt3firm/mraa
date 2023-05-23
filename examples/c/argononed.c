/*
 * Ported from argononed.py, https://github.com/Argon40Tech/Argon40case
 *
 * Runs on Radxa ROCK 4 C+ (and on Raspberry Pi 4B, also untested).
 */
#include <stddef.h>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <pthread.h>

#include "gpio.h"
#include "mraa.h"

/* I²C bus number, 7 on RasPi 4 and ROCK Pi 4 */
#define BUS_NUM  7

/* I²C address of the MCU */
#define ADDR_FAN 0x1A

/* Not-a-Number */
#define NaN (0.0f/0.0f)

/* strip a line */
static char *strip(char *line)
{
    char *s = line;
    for (; *s != '\0' && isblank(*s); ++s);
    if (*s == '\0') {
        return s;
    }

    char *e = s + strlen(s);

    for (; e > s && !isgraph(e[-1]); --e);
    *e = '\0';
    return s;
}

/* On entry in the fan config */
typedef struct config_entry {
    float temperature;  /* temperature threshold */
    int   fan_speed;    /* fan speed in % */
    struct config_entry *next;
} config_entry_t;

/* Put a new config entry into the ordered list */
static void append_config(config_entry_t **list, float temperature, int fan_speed)
{
    config_entry_t *e = malloc(sizeof(config_entry_t));
    if (e == NULL) {
        return;
    }
    e->temperature = temperature;
    e->fan_speed   = fan_speed;

    while (*list != NULL && (*list)->temperature > temperature) {
        list = &(*list)->next;
    }
    e->next = *list;
    *list = e;
}

/*
 * This function retrieves the fanspeed configuration list from a file, arranged by temperature
 * It ignores lines beginning with "#" and checks if the line is a valid temperature-speed pair
 * The temperature values are formatted to uniform length, so the lines can be sorted properly
 */
static config_entry_t *load_config(char const *fname)
{
	config_entry_t *newconfig = NULL;
	FILE *f = fopen(fname, "r");
    char *curline;
    size_t len = 0;
    if (f != NULL) {
        while (getline(&curline, &len, f) != -1) {
            char *tmpline = strip(curline);
            if (tmpline[0] == '\0') {
                continue;
            }
            if (tmpline[0] == '#') {
                continue;
            }
            char *assign = strchr(tmpline, '=');
            if (assign == NULL || strchr(assign + 1, '=') != NULL) {
                continue;
            }
            *assign = '\0';
            char *ptempval = strip(tmpline);
            char *pfanval  = strip(assign + 1);
            char *end = NULL;
            float tempval = strtof(ptempval, &end);
            if (tempval < 0.0f || tempval > 100.0f || *end != '\0') {
                continue;
            }
            int fanval = (int)strtof(pfanval, &end);
            if (fanval < 0 || fanval > 100 || *end != '\0') {
                continue;
            }
            append_config(&newconfig, tempval, fanval);
        }
    }
	return newconfig;
}

/* get current temperature in °C */
static float get_temp()
{
    char *line = NULL;
    size_t size = 0;

    FILE *f = fopen("/sys/class/thermal/thermal_zone0/temp", "r");
    if (f == NULL) {
        // could not read the temperature, return NaN
        return NaN;
    }

    ssize_t n = getline(&line, &size, f);
    fclose(f);

    if (n < 0) {
        // could not read
        return NaN;
    }
    line[n] = '\0';

    size_t temp = 0;
    for (char const *p = line; *p != '\0'; ++p) {
        if ('0' <= *p && *p <= '9') {
            temp = temp * 10 + *p - '0';
        } else if (*p == '\n') {
            break;
        } else {
            // unexpected format
            free(line);
            return NaN;
        }
    }
    free(line);

    return (float)temp / 1000.0f;
}

/* set fan speed */
static void set_fan_speed(mraa_i2c_context dev, int speed)
{
    if (speed < 0)
        speed = 0;
    if (speed > 100)
        speed = 100;

    /* set slave address */
    mraa_result_t status = mraa_i2c_address(dev, ADDR_FAN);
    if (status != MRAA_SUCCESS) {
        return;
    }
    status = mraa_i2c_write_byte(dev, (uint8_t)speed);
    if (status != MRAA_SUCCESS) {
        return;
    }
}

/* signal power off */
static void signal_poweroff(mraa_i2c_context dev)
{
    /* set slave address */
    mraa_result_t status = mraa_i2c_address(dev, ADDR_FAN);
    if (status != MRAA_SUCCESS) {
        return;
    }
    status = mraa_i2c_write_byte(dev, 0xFF);
    if (status != MRAA_SUCCESS) {
        return;
    }
}


/* turn fan off */
static void fan_off(mraa_i2c_context dev)
{
    set_fan_speed(dev, 0);
}

/* get the fan speed for the given temperature */
static int get_fan_speed(config_entry_t const *config, float temp)
{
    while (config != NULL && config->temperature > temp) {
        config = config->next;
    }
    return config != NULL ? config->fan_speed : 0;
}

/* default config: "65°C=100%", "60°C=55%", "55°C=10%"] */
static const config_entry_t ce_low  = { 55,  10, NULL };
static const config_entry_t ce_mid  = { 60,  55, (config_entry_t *)&ce_low };
static const config_entry_t ce_high = { 65, 100, (config_entry_t *)&ce_mid };
static const config_entry_t *default_config = &ce_high;

/* The fan daemon: check temperature and contriol the fan. */
static int check_fan_speed(mraa_i2c_context dev)
{
    config_entry_t const *config = load_config("/etc/argononed.conf");
    if (config == NULL) {
        config = default_config;
    }

    int prev_speed = -1;
    for (;;) {
        float temp = get_temp();

        if (!isnan(temp)) {
            int speed = get_fan_speed(config, temp);

            if (speed != prev_speed) {
                prev_speed = speed;
                
                if (speed > 0) {
                    // spin up to prevent issues on older units
                    set_fan_speed(dev, 100);
                }
                set_fan_speed(dev, speed);
            }
        }
        sleep(30);
    }
    return EXIT_SUCCESS;
}

static volatile int ready_flag = 0;
static pthread_mutex_t ready_mutex = PTHREAD_MUTEX_INITIALIZER;
static pthread_cond_t ready_cond = PTHREAD_COND_INITIALIZER;

/*
   This is the "interrupt routine". Unfurtunately it is called from a waiting thread and other mraa
   calls may fail while it is running, hence there is not much we can do here.
 */
static void button_isr(void *arg)
{
    pthread_mutex_lock(&ready_mutex);
    ready_flag = 1;

    pthread_cond_signal(&ready_cond);
    pthread_mutex_unlock(&ready_mutex);
}

/* wait for a raising edge on the GPIO */
static void wait_for_raising_edge(mraa_gpio_context dev)
{
    /* install an isr to wait for the condition */
    mraa_result_t result = mraa_gpio_isr(dev, MRAA_GPIO_EDGE_RISING, button_isr, NULL);
    if (result != MRAA_SUCCESS) {
        return;
    }

    /* wait for the isr */
    pthread_mutex_lock(&ready_mutex);
    while (!ready_flag) {
        pthread_cond_wait(&ready_cond, &ready_mutex);
    }
    ready_flag = 0;
    pthread_mutex_unlock(&ready_mutex);

    /* exit the isr: otherwise polling might fail */
    mraa_gpio_isr_exit(dev);
}

/* The reboot daemon: check the button and reboot */
static int check_shutdown(mraa_i2c_context unused)
{
    int const PIN_SHUTDOWN = 7;

    mraa_gpio_context dev = mraa_gpio_init(PIN_SHUTDOWN);
    if (dev == NULL) {
        return EXIT_FAILURE;
    }
    mraa_result_t result = mraa_gpio_dir(dev, MRAA_GPIO_IN);
    if (result != MRAA_SUCCESS) {
        return EXIT_FAILURE;
    }

    for (;;) {
        wait_for_raising_edge(dev);

        /* poll the pin */
        unsigned pulse_time = 1;
        for (;;) {
            int value = mraa_gpio_read(dev);
            if (value != 1) {
                break;
            }
            usleep(10000);
            ++pulse_time;
        };
        if (pulse_time >= 2 && pulse_time <= 3) {
			system("reboot");
        } else if (pulse_time >= 4 && pulse_time <= 5) {
			system("shutdown now -h");
        }
    }

    return EXIT_SUCCESS;
}

/* run func as a forked process */
static int run_daemon(int (*func)(mraa_i2c_context), mraa_i2c_context arg) {
    pid_t pid;
    pid = fork(); // Fork off the parent process
    if (pid < 0) {
        return EXIT_FAILURE;
    }
    if (pid > 0) {
        return EXIT_SUCCESS;
    }
    close(STDIN_FILENO);
    close(STDOUT_FILENO);
    close(STDERR_FILENO);

    return func(arg);
}

int main(int argc, char const *argv[])
{
    mraa_i2c_context dev = mraa_i2c_init_raw(BUS_NUM);

    if (dev == NULL) {
        return EXIT_FAILURE;
    }

    if (argc > 1) {
        if (strcasecmp(argv[1], "shutdown") == 0) {
            signal_poweroff(dev);
            return EXIT_SUCCESS;
        } else if (strcasecmp(argv[1], "fanoff") == 0) {
            fan_off(dev);
            return EXIT_SUCCESS;
        } else if (strcasecmp(argv[1], "service") == 0) {
            run_daemon(check_fan_speed, dev);
            run_daemon(check_shutdown, NULL);
            return EXIT_SUCCESS;
        }
    }

    printf("%s: [shutdown|fanoff|service]\n", argv[0]);
    return EXIT_SUCCESS;
}