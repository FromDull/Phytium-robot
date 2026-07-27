#define _POSIX_C_SOURCE 200809L

#include "../src/rpmsg_protocol.h"
#include "../src/rpmsg_transport.h"

#include <errno.h>
#include <fcntl.h>
#include <getopt.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/select.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <time.h>
#include <unistd.h>

#define LOGGER_VERSION "1.1.0"
#define DEFAULT_DEVICE "/dev/rpmsg0"
#define DEFAULT_LOG_DIR "logs/balance"
#define DEFAULT_RATE_HZ 20U
#define MAX_RATE_HZ 50U

typedef struct {
    uint8_t state;
    uint8_t fault;
    uint16_t control_hz;
    uint32_t loop_count;
    int32_t pitch_x1e6;
    int32_t pitch_rate_x1e6;
    int32_t position_x1e6;
    int32_t velocity_x1e6;
    int32_t left_torque_x1e6;
    int32_t right_torque_x1e6;
    int16_t left_current_x100;
    int16_t right_current_x100;
    int16_t left_rpm;
    int16_t right_rpm;
    int32_t pitch_target_x1e6;
    int32_t position_target_x1e6;
    int32_t position_error_x1e6;
    int32_t velocity_error_x1e6;
    uint8_t position_hold_enabled;
} BalanceSample;

static volatile sig_atomic_t g_stop;

static void handle_signal(int signo)
{
    (void)signo;
    g_stop = 1;
}

static uint16_t read_be_u16(const uint8_t *p)
{
    return (uint16_t)(((uint16_t)p[0] << 8) | p[1]);
}

static uint32_t read_be_u32(const uint8_t *p)
{
    return ((uint32_t)p[0] << 24) |
           ((uint32_t)p[1] << 16) |
           ((uint32_t)p[2] << 8) |
           (uint32_t)p[3];
}

static int32_t read_be_i32(const uint8_t *p)
{
    return (int32_t)read_be_u32(p);
}

static int wait_readable(int fd, int timeout_ms)
{
    fd_set read_fds;
    struct timeval timeout;

    FD_ZERO(&read_fds);
    FD_SET(fd, &read_fds);
    timeout.tv_sec = timeout_ms / 1000;
    timeout.tv_usec = (timeout_ms % 1000) * 1000;
    return select(fd + 1, &read_fds, NULL, NULL, &timeout);
}

static int exchange_command(int fd, uint8_t type, uint8_t seq,
                            RpmsgFrame *reply, int timeout_ms)
{
    uint8_t tx[16];
    uint8_t rx[128];
    size_t tx_size = rpmsg_encode(type, seq, NULL, 0, tx, sizeof(tx));
    ssize_t rx_size;
    int ready;

    if (tx_size == 0U || write(fd, tx, tx_size) != (ssize_t)tx_size) {
        return -1;
    }
    ready = wait_readable(fd, timeout_ms);
    if (ready <= 0) {
        return ready == 0 ? -2 : -1;
    }
    rx_size = read(fd, rx, sizeof(rx));
    if (rx_size <= 0 || !rpmsg_decode(rx, (size_t)rx_size, reply) ||
        reply->seq != seq) {
        return -3;
    }
    return 0;
}

static int decode_sample(const RpmsgFrame *reply, BalanceSample *sample)
{
    const uint8_t *p = reply->payload;

    if (reply->type != CMD_BALANCE_TELEMETRY ||
        !((p[0] == 1U && reply->length == 44U) ||
          (p[0] == BALANCE_TELEMETRY_VERSION &&
           reply->length == BALANCE_TELEMETRY_PAYLOAD_SIZE))) {
        return -1;
    }
    memset(sample, 0, sizeof(*sample));
    sample->state = p[1];
    sample->fault = p[2];
    sample->control_hz = read_be_u16(&p[4]);
    sample->loop_count = read_be_u32(&p[8]);
    sample->pitch_x1e6 = read_be_i32(&p[12]);
    sample->pitch_rate_x1e6 = read_be_i32(&p[16]);
    sample->position_x1e6 = read_be_i32(&p[20]);
    sample->velocity_x1e6 = read_be_i32(&p[24]);
    sample->left_torque_x1e6 = read_be_i32(&p[28]);
    sample->right_torque_x1e6 = read_be_i32(&p[32]);
    sample->left_current_x100 = (int16_t)read_be_u16(&p[36]);
    sample->right_current_x100 = (int16_t)read_be_u16(&p[38]);
    sample->left_rpm = (int16_t)read_be_u16(&p[40]);
    sample->right_rpm = (int16_t)read_be_u16(&p[42]);
    if (p[0] >= 2U && reply->length >= 60U) {
        sample->position_hold_enabled = p[3];
        sample->pitch_target_x1e6 = read_be_i32(&p[44]);
        sample->position_target_x1e6 = read_be_i32(&p[48]);
        sample->position_error_x1e6 = read_be_i32(&p[52]);
        sample->velocity_error_x1e6 = read_be_i32(&p[56]);
    }
    return 0;
}

static int64_t timespec_diff_us(const struct timespec *end,
                                const struct timespec *start)
{
    return (int64_t)(end->tv_sec - start->tv_sec) * 1000000LL +
           (int64_t)(end->tv_nsec - start->tv_nsec) / 1000LL;
}

static void timespec_add_ns(struct timespec *value, int64_t nanoseconds)
{
    value->tv_nsec += nanoseconds;
    while (value->tv_nsec >= 1000000000L) {
        value->tv_nsec -= 1000000000L;
        value->tv_sec++;
    }
}

static int timespec_not_after(const struct timespec *left,
                              const struct timespec *right)
{
    return left->tv_sec < right->tv_sec ||
           (left->tv_sec == right->tv_sec && left->tv_nsec <= right->tv_nsec);
}

static void make_timestamp(const struct timespec *realtime, char *buffer,
                           size_t buffer_size)
{
    struct tm local;
    char date[32];
    char zone[16];

    localtime_r(&realtime->tv_sec, &local);
    strftime(date, sizeof(date), "%Y-%m-%dT%H:%M:%S", &local);
    strftime(zone, sizeof(zone), "%z", &local);
    snprintf(buffer, buffer_size, "%s.%03ld%s", date,
             realtime->tv_nsec / 1000000L, zone);
}

static int mkdir_recursive(const char *path)
{
    char copy[512];

    if (strlen(path) >= sizeof(copy)) {
        return -1;
    }
    strcpy(copy, path);
    for (char *p = copy + 1; *p != '\0'; ++p) {
        if (*p == '/') {
            *p = '\0';
            if (mkdir(copy, 0755) != 0 && errno != EEXIST) {
                return -1;
            }
            *p = '/';
        }
    }
    return mkdir(copy, 0755) == 0 || errno == EEXIST ? 0 : -1;
}

static int drop_sudo_privileges(void)
{
    const char *uid_text = getenv("SUDO_UID");
    const char *gid_text = getenv("SUDO_GID");

    if (geteuid() != 0 || uid_text == NULL || gid_text == NULL) {
        return 0;
    }
    if (setgid((gid_t)strtoul(gid_text, NULL, 10)) != 0 ||
        setuid((uid_t)strtoul(uid_text, NULL, 10)) != 0) {
        return -1;
    }
    return 0;
}

static const char *state_name(uint8_t state)
{
    static const char *const names[] = {
        "disabled", "arming", "active", "fault"
    };
    return state < 4U ? names[state] : "unknown";
}

static void gzip_file_async(const char *path)
{
    pid_t child;

    if (path[0] == '\0') {
        return;
    }
    child = fork();
    if (child == 0) {
        execlp("gzip", "gzip", "-f", path, (char *)NULL);
        _exit(127);
    }
}

static FILE *open_hour_file(const char *directory, const char *run_id,
                            const struct timespec *realtime, char *hour_key,
                            size_t hour_key_size, char *path, size_t path_size)
{
    struct tm local;
    FILE *file;

    localtime_r(&realtime->tv_sec, &local);
    strftime(hour_key, hour_key_size, "%Y%m%d_%H", &local);
    snprintf(path, path_size, "%s/balance_%s_%s.csv", directory, run_id,
             hour_key);
    file = fopen(path, "w");
    if (file != NULL) {
        fputs("timestamp,epoch_ms,sample_index,read_ok,latency_us,state,"
              "state_id,fault,control_hz,loop_count,pitch_rad,"
              "pitch_rate_rad_s,position_m,velocity_m_s,left_torque_nm,"
              "right_torque_nm,left_current_a,right_current_a,left_rpm,"
              "right_rpm,position_hold_enabled,pitch_target_rad,"
              "position_target_m,position_error_m,velocity_error_m_s\n", file);
    }
    return file;
}

static void print_usage(const char *program)
{
    printf("balance_logger %s\n", LOGGER_VERSION);
    printf("Usage: %s [--device PATH] [--rate HZ] [--output-dir DIR]\n",
           program);
    printf("          [--enable] [--duration SEC] [--stop-on-fault]\n");
    printf("\nExamples:\n");
    printf("  sudo %s --rate 20 --enable\n", program);
    printf("  sudo %s --rate 50 --duration 30 --enable\n", program);
}

int main(int argc, char **argv)
{
    static const struct option options[] = {
        {"device", required_argument, NULL, 'd'},
        {"rate", required_argument, NULL, 'r'},
        {"output-dir", required_argument, NULL, 'o'},
        {"enable", no_argument, NULL, 'e'},
        {"duration", required_argument, NULL, 't'},
        {"stop-on-fault", no_argument, NULL, 'f'},
        {"help", no_argument, NULL, 'h'},
        {NULL, 0, NULL, 0}
    };
    const char *device = getenv("RPMSG_DEVICE");
    const char *output_dir = DEFAULT_LOG_DIR;
    unsigned int rate_hz = DEFAULT_RATE_HZ;
    double duration_s = 0.0;
    int enable = 0;
    int stop_on_fault = 0;
    int fd;
    int option;
    int64_t period_ns;
    uint8_t seq = 1U;
    uint64_t sample_index = 0U;
    uint64_t success_count = 0U;
    uint64_t error_count = 0U;
    uint64_t summary_samples = 0U;
    char run_id[32];
    char current_hour[16] = "";
    char current_path[1024] = "";
    FILE *file = NULL;
    struct timespec start_mono;
    struct timespec next_sample;
    struct timespec last_summary;

    if (device == NULL || device[0] == '\0') {
        device = DEFAULT_DEVICE;
    }
    while ((option = getopt_long(argc, argv, "d:r:o:et:fh", options,
                                 NULL)) != -1) {
        switch (option) {
        case 'd': device = optarg; break;
        case 'r': rate_hz = (unsigned int)strtoul(optarg, NULL, 10); break;
        case 'o': output_dir = optarg; break;
        case 'e': enable = 1; break;
        case 't': duration_s = strtod(optarg, NULL); break;
        case 'f': stop_on_fault = 1; break;
        case 'h': print_usage(argv[0]); return 0;
        default: print_usage(argv[0]); return 1;
        }
    }
    if (rate_hz == 0U || rate_hz > MAX_RATE_HZ || duration_s < 0.0) {
        fprintf(stderr, "rate must be 1..%u Hz and duration must be >= 0\n",
                MAX_RATE_HZ);
        return 1;
    }

    fd = rpmsg_transport_open(device);
    if (fd < 0) {
        perror("connect rpmsg broker");
        return 1;
    }
    if (drop_sudo_privileges() != 0) {
        perror("drop sudo privileges");
        close(fd);
        return 1;
    }
    if (mkdir_recursive(output_dir) != 0) {
        perror("create log directory");
        close(fd);
        return 1;
    }

    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    period_ns = 1000000000LL / (int64_t)rate_hz;
    clock_gettime(CLOCK_MONOTONIC, &start_mono);
    next_sample = start_mono;
    last_summary = start_mono;
    {
        struct timespec realtime;
        struct tm local;
        clock_gettime(CLOCK_REALTIME, &realtime);
        localtime_r(&realtime.tv_sec, &local);
        strftime(run_id, sizeof(run_id), "%Y%m%d_%H%M%S", &local);
    }

    if (enable) {
        RpmsgFrame reply;
        if (exchange_command(fd, CMD_BALANCE_ENABLE, seq++, &reply, 3000) != 0) {
            fprintf(stderr, "failed to enable balance controller\n");
            close(fd);
            return 1;
        }
    }
    printf("balance_logger %s: device=%s rate=%u Hz enable=%s\n",
           LOGGER_VERSION, device, rate_hz, enable ? "yes" : "no");

    while (!g_stop) {
        struct timespec request_start;
        struct timespec request_end;
        struct timespec realtime;
        RpmsgFrame reply;
        BalanceSample sample;
        char timestamp[64];
        char hour[16];
        int read_result;
        int64_t latency_us;
        int64_t epoch_ms;

        clock_gettime(CLOCK_REALTIME, &realtime);
        {
            struct tm local;
            localtime_r(&realtime.tv_sec, &local);
            strftime(hour, sizeof(hour), "%Y%m%d_%H", &local);
        }
        if (file == NULL || strcmp(hour, current_hour) != 0) {
            char previous_path[sizeof(current_path)];
            strcpy(previous_path, current_path);
            if (file != NULL) {
                fclose(file);
                file = NULL;
            }
            if (previous_path[0] != '\0') {
                gzip_file_async(previous_path);
            }
            file = open_hour_file(output_dir, run_id, &realtime,
                                  current_hour, sizeof(current_hour),
                                  current_path, sizeof(current_path));
            if (file == NULL) {
                perror("open log file");
                break;
            }
            printf("logging to %s\n", current_path);
        }

        clock_gettime(CLOCK_MONOTONIC, &request_start);
        read_result = exchange_command(fd, CMD_BALANCE_TELEMETRY, seq++,
                                       &reply, 200);
        clock_gettime(CLOCK_MONOTONIC, &request_end);
        latency_us = timespec_diff_us(&request_end, &request_start);
        if (read_result == 0) {
            read_result = decode_sample(&reply, &sample);
        }
        clock_gettime(CLOCK_REALTIME, &realtime);
        make_timestamp(&realtime, timestamp, sizeof(timestamp));
        epoch_ms = (int64_t)realtime.tv_sec * 1000LL +
                   realtime.tv_nsec / 1000000L;

        if (read_result == 0) {
            fprintf(file,
                    "%s,%lld,%llu,1,%lld,%s,%u,0x%02X,%u,%u,"
                    "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.2f,%.2f,%d,%d,"
                    "%u,%.6f,%.6f,%.6f,%.6f\n",
                    timestamp, (long long)epoch_ms,
                    (unsigned long long)sample_index,
                    (long long)latency_us, state_name(sample.state),
                    sample.state, sample.fault, sample.control_hz,
                    sample.loop_count, sample.pitch_x1e6 / 1000000.0,
                    sample.pitch_rate_x1e6 / 1000000.0,
                    sample.position_x1e6 / 1000000.0,
                    sample.velocity_x1e6 / 1000000.0,
                    sample.left_torque_x1e6 / 1000000.0,
                    sample.right_torque_x1e6 / 1000000.0,
                    sample.left_current_x100 / 100.0,
                    sample.right_current_x100 / 100.0,
                    sample.left_rpm, sample.right_rpm,
                    sample.position_hold_enabled,
                    sample.pitch_target_x1e6 / 1000000.0,
                    sample.position_target_x1e6 / 1000000.0,
                    sample.position_error_x1e6 / 1000000.0,
                    sample.velocity_error_x1e6 / 1000000.0);
            success_count++;
            if (stop_on_fault && sample.state == 3U) {
                printf("fault detected: 0x%02X\n", sample.fault);
                g_stop = 1;
            }
        } else {
            fprintf(file,
                    "%s,%lld,%llu,0,%lld,read_error,-1,unknown,0,0,"
                    "nan,nan,nan,nan,nan,nan,nan,nan,nan,nan,"
                    "nan,nan,nan,nan,nan\n",
                    timestamp, (long long)epoch_ms,
                    (unsigned long long)sample_index,
                    (long long)latency_us);
            error_count++;
        }
        sample_index++;
        summary_samples++;

        if (timespec_diff_us(&request_end, &last_summary) >= 1000000LL) {
            double elapsed = timespec_diff_us(&request_end, &last_summary) /
                             1000000.0;
            printf("rate=%.1f Hz samples=%llu errors=%llu",
                   summary_samples / elapsed,
                   (unsigned long long)success_count,
                   (unsigned long long)error_count);
            if (read_result == 0) {
                printf(" state=%s fault=0x%02X pitch=%.3f deg vel=%.3f m/s target=%.3f deg pos_err=%.3f m",
                       state_name(sample.state), sample.fault,
                       sample.pitch_x1e6 / 1000000.0 * 57.295779513,
                       sample.velocity_x1e6 / 1000000.0,
                       sample.pitch_target_x1e6 / 1000000.0 * 57.295779513,
                       sample.position_error_x1e6 / 1000000.0);
            }
            putchar('\n');
            fflush(stdout);
            fflush(file);
            last_summary = request_end;
            summary_samples = 0U;
            while (waitpid(-1, NULL, WNOHANG) > 0) {
            }
        }
        if (duration_s > 0.0 &&
            timespec_diff_us(&request_end, &start_mono) >=
                (int64_t)(duration_s * 1000000.0)) {
            break;
        }

        timespec_add_ns(&next_sample, period_ns);
        if (timespec_not_after(&next_sample, &request_end)) {
            next_sample = request_end;
            timespec_add_ns(&next_sample, period_ns);
        }
        if (clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME,
                            &next_sample, NULL) != 0) {
            clock_gettime(CLOCK_MONOTONIC, &next_sample);
        }
    }

    if (enable) {
        RpmsgFrame reply;
        (void)exchange_command(fd, CMD_BALANCE_DISABLE, seq++, &reply, 500);
    }
    if (file != NULL) {
        fflush(file);
        fclose(file);
    }
    close(fd);
    printf("stopped: samples=%llu errors=%llu file=%s\n",
           (unsigned long long)success_count,
           (unsigned long long)error_count, current_path);
    return error_count == sample_index ? 2 : 0;
}
