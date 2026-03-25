/*
 * USRP SDR backend for iridium-sniffer
 * Based on ice9-bluetooth-sniffer (Copyright 2022 ICE9 Consulting LLC)
 */

#include <err.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>

#include <uhd.h>

#include "sdr.h"

extern sig_atomic_t running;
extern pid_t self_pid;
extern double samp_rate;
extern double center_freq;
extern int usrp_gain_val;
extern int clock_source;
extern int time_source;
extern int usrp_pps_ref;
extern int verbose;

#define KVLEN 16
typedef struct _kv_pair_t {
    char key[KVLEN];
    char value[KVLEN];
} kv_pair_t;

kv_pair_t *parse_kv_pairs(char *str, unsigned *pairs_out) {
    char *cur = str, *end = str + strlen(str);
    char *comma = NULL, *equals = NULL;

    unsigned pair = 0;
    kv_pair_t *ret;

    while (cur < end) {
        ++pair;
        if ((comma = strchr(cur, ',')) != NULL)
            cur = comma + 1;
        else
            break;
    }

    ret = calloc(pair, sizeof(kv_pair_t));
    pair = 0;
    cur = str;
    while (cur < end) {
        if ((comma = strchr(cur, ',')) != NULL)
            *comma = 0;
        equals = strchr(cur, '=');
        if (equals == NULL) {
            free(ret);
            return NULL;
        }
        *equals = 0;
        strncpy(ret[pair].key, cur, KVLEN-1);
        ret[pair].key[KVLEN-1] = 0;
        strncpy(ret[pair].value, equals + 1, KVLEN-1);
        ret[pair].value[KVLEN-1] = 0;
        ++pair;
        if (comma == NULL)
            break;
        else
            cur = comma + 1;
    }

    *pairs_out = pair;
    return ret;
}

static char *_find(char *key, kv_pair_t *p, unsigned num_pairs) {
    unsigned i;
    for (i = 0; i < num_pairs; ++i)
        if (strcmp(p[i].key, key) == 0)
            return p[i].value;
    return NULL;
}

void usrp_list(void) {
    char buf[128];
    size_t n;
    kv_pair_t *usrp_info;
    unsigned i, pairs = 0;
    char *product, *type, *serial;
    uhd_string_vector_handle devices_str;

    uhd_string_vector_make(&devices_str);
    uhd_usrp_find("", &devices_str);
    uhd_string_vector_size(devices_str, &n);

    for (i = 0; i < n; ++i) {
        uhd_string_vector_at(devices_str, i, buf, sizeof(buf));
        usrp_info = parse_kv_pairs(buf, &pairs);

        product = _find("product", usrp_info, pairs);
        if (product == NULL)
            product = "unk";

        type = _find("type", usrp_info, pairs);
        serial = _find("serial", usrp_info, pairs);
        if (type == NULL || serial == NULL) {
            free(usrp_info);
            continue;
        }

        {
            char val[128];
            snprintf(val, sizeof(val), "usrp-%s-%s", product, serial);
            printf("  %-24s USRP %s\n", val, product);
        }

        free(usrp_info);
    }

    uhd_string_vector_free(&devices_str);
}

char *usrp_get_serial(char *name) {
    char *after_prefix, *dash;
    if (strncmp(name, "usrp-", 5) != 0)
        return NULL;
    after_prefix = name + 5;
    dash = strchr(after_prefix, '-');
    if (dash == NULL)
        return NULL;
    return dash + 1;
}

uhd_usrp_handle usrp_setup(char *serial) {
    uhd_usrp_handle usrp;
    uhd_error error;
    char arg[128];
    uhd_tune_request_t tune_request = {
        .target_freq = center_freq,
        .rf_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO,
        .dsp_freq_policy = UHD_TUNE_REQUEST_POLICY_AUTO,
    };
    uhd_tune_result_t tune_result;

    snprintf(arg, sizeof(arg), "serial=%s,num_recv_frames=1024", serial);

    error = uhd_usrp_make(&usrp, arg);
    if (error)
        errx(1, "Error opening UHD: %u", error);

    /* Configure clock source */
    if (clock_source == CLOCK_SRC_EXTERNAL) {
        if ((error = uhd_usrp_set_clock_source(usrp, "external", 0)) != UHD_ERROR_NONE)
            errx(1, "Unable to set USRP clock source to external: %u", error);
        if (verbose)
            fprintf(stderr, "USRP: clock source set to external\n");
    } else if (clock_source == CLOCK_SRC_GPSDO) {
        if ((error = uhd_usrp_set_clock_source(usrp, "gpsdo", 0)) != UHD_ERROR_NONE)
            errx(1, "Unable to set USRP clock source to gpsdo: %u", error);
        if (verbose)
            fprintf(stderr, "USRP: clock source set to gpsdo\n");
    }

    /* Configure time source */
    if (time_source == CLOCK_SRC_EXTERNAL) {
        if ((error = uhd_usrp_set_time_source(usrp, "external", 0)) != UHD_ERROR_NONE)
            errx(1, "Unable to set USRP time source to external: %u", error);
        if (verbose)
            fprintf(stderr, "USRP: time source set to external\n");
    } else if (time_source == CLOCK_SRC_GPSDO) {
        if ((error = uhd_usrp_set_time_source(usrp, "gpsdo", 0)) != UHD_ERROR_NONE)
            errx(1, "Unable to set USRP time source to gpsdo: %u", error);
        if (verbose)
            fprintf(stderr, "USRP: time source set to gpsdo\n");
    }

    /* If --pps-ref was requested, wait for the PPS reference to lock and then
     * set the device time at the next PPS pulse so that RX metadata timestamps
     * represent real wall-clock nanoseconds.  The output format is unchanged;
     * downstream code already converts hardware timestamps to relative ms. */
    if (usrp_pps_ref) {
        /* Candidates to check; actual availability depends on hardware. */
        static const char * const lock_sensor_candidates[] = {
            "pps_locked", "gps_locked", "ref_locked", NULL
        };
        const int max_tries = 30;   /* 30 s timeout */
        bool locked = false;

        /* Calling get_mboard_sensor with an unknown sensor name causes a
         * SIGSEGV inside some UHD versions, so enumerate the sensors that
         * actually exist on this device before querying any of them. */
        const char *available[4] = {NULL};
        int n_available = 0;
        uhd_string_vector_handle sensor_names = NULL;
        if (uhd_string_vector_make(&sensor_names) == UHD_ERROR_NONE &&
                uhd_usrp_get_mboard_sensor_names(usrp, 0, &sensor_names) == UHD_ERROR_NONE) {
            size_t n = 0;
            uhd_string_vector_size(sensor_names, &n);
            char buf[64];
            for (size_t i = 0; i < n; ++i) {
                if (uhd_string_vector_at(sensor_names, i, buf, sizeof(buf)) != UHD_ERROR_NONE)
                    continue;
                for (int ci = 0; lock_sensor_candidates[ci]; ++ci) {
                    if (strcmp(buf, lock_sensor_candidates[ci]) == 0) {
                        available[n_available++] = lock_sensor_candidates[ci];
                        break;
                    }
                }
            }
        }
        if (sensor_names)
            uhd_string_vector_free(&sensor_names);

        if (n_available == 0) {
            if (verbose)
                fprintf(stderr, "USRP: no lock sensors found on device; "
                        "skipping PPS wait\n");
        } else {
            if (verbose)
                fprintf(stderr, "USRP: waiting for PPS/reference lock (max %d s)...\n",
                        max_tries);
            for (int attempt = 0; attempt < max_tries && !locked; ++attempt) {
                for (int si = 0; si < n_available && !locked; ++si) {
                    uhd_sensor_value_handle sv = NULL;
                    if (uhd_usrp_get_mboard_sensor(usrp, available[si], 0, &sv)
                            == UHD_ERROR_NONE) {
                        bool b = false;
                        uhd_sensor_value_to_bool(sv, &b);
                        uhd_sensor_value_free(&sv);
                        if (b) {
                            locked = true;
                            if (verbose)
                                fprintf(stderr, "USRP: %s asserted after %d s\n",
                                        available[si], attempt);
                        }
                    }
                }
                if (!locked)
                    sleep(1);
            }
            if (!locked)
                warnx("USRP: PPS/reference lock timed out; timestamps may be inaccurate");
        }

        /* Set device time at the next PPS edge.  Use current system time + 1 s
         * so that the full-seconds counter sent to set_time_next_pps is the
         * integer second that will roll over on the upcoming PPS pulse. */
        time_t now = time(NULL);
        if ((error = uhd_usrp_set_time_next_pps(usrp, (int64_t)(now + 1), 0.0, 0))
                != UHD_ERROR_NONE)
            warnx("USRP: set_time_next_pps failed: %u (timestamps may be inaccurate)",
                  error);
        else {
            /* Wait one more second so the PPS latch has definitely occurred
             * before streaming starts. */
            sleep(2);
            if (verbose)
                fprintf(stderr, "USRP: device time set at next PPS (epoch %ld)\n",
                        (long)(now + 1));
        }
    }

    if ((error = uhd_usrp_set_rx_rate(usrp, samp_rate, 0)) != UHD_ERROR_NONE)
        errx(1, "Unable to set USRP sample rate: %u", error);

    /* Read back the rate UHD actually committed to (may differ from requested).
     * All downstream buffers are sized from samp_rate, so this MUST be updated
     * before burst_detector_create() or any other sizing call is made.
     * Mismatch (e.g. B200 snapping 10 MHz -> 16 MHz) causes heap corruption. */
    {
        double actual_rate = 0;
        if (uhd_usrp_get_rx_rate(usrp, 0, &actual_rate) == UHD_ERROR_NONE
                && actual_rate > 0) {
            if (actual_rate != samp_rate) {
                fprintf(stderr, "USRP: requested %.6g MHz, got %.6g MHz — "
                        "updating samp_rate\n",
                        samp_rate / 1e6, actual_rate / 1e6);
                samp_rate = actual_rate;
            }
        }
    }

    if ((error = uhd_usrp_set_rx_gain(usrp, (double)usrp_gain_val, 0, "")) != UHD_ERROR_NONE)
        errx(1, "Unable to set USRP gain: %u", error);
    if ((error = uhd_usrp_set_rx_freq(usrp, &tune_request, 0, &tune_result)) != UHD_ERROR_NONE)
        errx(1, "Unable to set USRP frequency: %u", error);

    return usrp;
}

void *usrp_stream_thread(void *arg) {
    uhd_usrp_handle usrp = arg;
    uhd_rx_streamer_handle rx_handle;
    uhd_error error;
    uhd_rx_metadata_handle md;
    size_t channel = 0, num_samples, num_rx_samples;
    void *buf;
    uhd_rx_metadata_error_code_t error_code;
    uhd_stream_args_t stream_args = {
        .cpu_format = "sc8",
        .otw_format = "sc8",
        .args = "",
        .channel_list = &channel,
        .n_channels = 1
    };
    uhd_stream_cmd_t stream_cmd = {
        .stream_mode = UHD_STREAM_MODE_START_CONTINUOUS,
        .stream_now = 1,
    };

    uhd_rx_metadata_make(&md);

    uhd_rx_streamer_make(&rx_handle);
    stream_args.channel_list = &channel;
    stream_args.n_channels = 1;
    error = uhd_usrp_get_rx_stream(usrp, &stream_args, rx_handle);
    if (error)
        errx(1, "Error opening RX stream: %u", error);

    uhd_rx_streamer_max_num_samps(rx_handle, &num_samples);
    uhd_rx_streamer_issue_stream_cmd(rx_handle, &stream_cmd);

    int hw_time = (time_source != CLOCK_SRC_INTERNAL);

    while (running) {
        sample_buf_t *s = malloc(sizeof(*s) + num_samples * 2 * sizeof(int8_t));
        s->format = SAMPLE_FMT_INT8;
        s->hw_timestamp_ns = 0;
        buf = s->samples;
        uhd_rx_streamer_recv(rx_handle, &buf, num_samples, &md, 3.0, false, &num_rx_samples);
        uhd_rx_metadata_error_code(md, &error_code);
        if (error_code != UHD_RX_METADATA_ERROR_CODE_NONE && error_code != 8)
            errx(1, "Error during streaming: %u", error_code);
        s->num = num_rx_samples;

        /* Extract hardware timestamp when time source is configured */
        if (hw_time) {
            int64_t full_secs;
            double frac_secs;
            uhd_rx_metadata_time_spec(md, &full_secs, &frac_secs);
            if (full_secs > 0)
                s->hw_timestamp_ns = (uint64_t)full_secs * 1000000000ULL
                                   + (uint64_t)(frac_secs * 1e9);
        }

        if (running)
            push_samples(s);
        else
            free(s);
    }

    stream_cmd.stream_mode = UHD_STREAM_MODE_STOP_CONTINUOUS;
    uhd_rx_streamer_issue_stream_cmd(rx_handle, &stream_cmd);

    uhd_rx_streamer_free(&rx_handle);
    uhd_rx_metadata_free(&md);

    return NULL;
}

void usrp_close(uhd_usrp_handle usrp) {
    uhd_usrp_free(&usrp);
}
