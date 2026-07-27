#define _GNU_SOURCE

#include "../src/rpmsg_protocol.h"
#include "../src/rpmsg_transport.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <pthread.h>
#include <signal.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/stat.h>
#include <sys/un.h>
#include <time.h>
#include <unistd.h>

#define BROKER_VERSION "1.1.0"
#define DEFAULT_DEVICE "/dev/rpmsg0"
#define TRANSACTION_TIMEOUT_MS 5000
#define MAX_MONITOR_CLIENTS 16

typedef struct {
    int client_fd;
} ClientContext;

static int g_device_fd = -1;
static int g_server_fd = -1;
static int g_monitor_server_fd = -1;
static int g_monitor_clients[MAX_MONITOR_CLIENTS];
static uint8_t g_sequence;
static volatile sig_atomic_t g_stopping;
static pthread_mutex_t g_device_lock = PTHREAD_MUTEX_INITIALIZER;
static pthread_mutex_t g_monitor_lock = PTHREAD_MUTEX_INITIALIZER;
static uint64_t g_event_id;
static uint64_t g_tx_frames;
static uint64_t g_rx_frames;
static uint64_t g_error_frames;
static uint64_t g_tx_bytes;
static uint64_t g_rx_bytes;

static int64_t monotonic_ms(void)
{
    struct timespec now;
    clock_gettime(CLOCK_MONOTONIC, &now);
    return (int64_t)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

static int64_t realtime_ms(void)
{
    struct timespec now;
    clock_gettime(CLOCK_REALTIME, &now);
    return (int64_t)now.tv_sec * 1000 + now.tv_nsec / 1000000;
}

static void bytes_to_hex(const uint8_t *data, size_t size,
                         char *output, size_t output_size)
{
    static const char digits[] = "0123456789ABCDEF";
    size_t count = size;

    if (output_size == 0U) {
        return;
    }
    if (count > (output_size - 1U) / 2U) {
        count = (output_size - 1U) / 2U;
    }
    for (size_t index = 0; index < count; ++index) {
        output[index * 2U] = digits[data[index] >> 4U];
        output[index * 2U + 1U] = digits[data[index] & 0x0fU];
    }
    output[count * 2U] = '\0';
}

static void monitor_broadcast(const char *message, size_t length)
{
    pthread_mutex_lock(&g_monitor_lock);
    for (size_t index = 0; index < MAX_MONITOR_CLIENTS; ++index) {
        if (g_monitor_clients[index] < 0) {
            continue;
        }
        if (send(g_monitor_clients[index], message, length,
                 MSG_DONTWAIT | MSG_NOSIGNAL) != (ssize_t)length) {
            if (errno != EAGAIN && errno != EWOULDBLOCK) {
                close(g_monitor_clients[index]);
                g_monitor_clients[index] = -1;
            }
        }
    }
    pthread_mutex_unlock(&g_monitor_lock);
}

static void monitor_emit(const char *direction, const RpmsgFrame *frame,
                         const uint8_t *wire_data, size_t wire_size,
                         uint8_t client_sequence, uint8_t wire_sequence,
                         int64_t latency_ms,
                         const char *status)
{
    char hex[(RPMSG_MAX_PAYLOAD + 5U) * 2U + 1U];
    char message[1024];
    int length;

    bytes_to_hex(wire_data, wire_size, hex, sizeof(hex));
    length = snprintf(
        message, sizeof(message),
        "{\"event_id\":%llu,\"epoch_ms\":%lld,\"direction\":\"%s\","
        "\"type\":%u,\"client_seq\":%u,\"wire_seq\":%u,"
        "\"payload_len\":%u,\"wire_bytes\":%zu,\"latency_ms\":%lld,"
        "\"status\":\"%s\",\"frame_hex\":\"%s\","
        "\"totals\":{\"tx_frames\":%llu,\"rx_frames\":%llu,"
        "\"errors\":%llu,\"tx_bytes\":%llu,\"rx_bytes\":%llu}}",
        (unsigned long long)++g_event_id, (long long)realtime_ms(), direction,
        frame != NULL ? frame->type : 0U, client_sequence, wire_sequence,
        frame != NULL ? frame->length : 0U, wire_size,
        (long long)latency_ms, status, hex,
        (unsigned long long)g_tx_frames, (unsigned long long)g_rx_frames,
        (unsigned long long)g_error_frames, (unsigned long long)g_tx_bytes,
        (unsigned long long)g_rx_bytes);
    if (length > 0 && (size_t)length < sizeof(message)) {
        monitor_broadcast(message, (size_t)length);
    }
}

static void *accept_monitor_clients(void *argument)
{
    (void)argument;
    while (!g_stopping) {
        int client_fd = accept4(g_monitor_server_fd, NULL, NULL,
                                SOCK_CLOEXEC | SOCK_NONBLOCK);
        if (client_fd < 0) {
            if (errno == EINTR || g_stopping) {
                continue;
            }
            break;
        }
        pthread_mutex_lock(&g_monitor_lock);
        size_t index;
        for (index = 0; index < MAX_MONITOR_CLIENTS; ++index) {
            if (g_monitor_clients[index] < 0) {
                g_monitor_clients[index] = client_fd;
                break;
            }
        }
        pthread_mutex_unlock(&g_monitor_lock);
        if (index == MAX_MONITOR_CLIENTS) {
            close(client_fd);
        }
    }
    return NULL;
}

static void request_stop(int signo)
{
    (void)signo;
    g_stopping = 1;
    if (g_server_fd >= 0) {
        close(g_server_fd);
        g_server_fd = -1;
    }
    if (g_monitor_server_fd >= 0) {
        close(g_monitor_server_fd);
        g_monitor_server_fd = -1;
    }
}

static int transact(const uint8_t *request_data, size_t request_size,
                    uint8_t *response_data, size_t response_capacity,
                    size_t *response_size)
{
    uint8_t tx[RPMSG_MAX_PAYLOAD + 5U];
    uint8_t rx[RPMSG_MAX_PAYLOAD + 5U];
    RpmsgFrame request;
    RpmsgFrame response;
    uint8_t broker_sequence;
    size_t tx_size;
    int64_t transaction_start;
    int64_t deadline;
    int result = -1;

    if (!rpmsg_decode(request_data, request_size, &request)) {
        errno = EPROTO;
        return -1;
    }
    pthread_mutex_lock(&g_device_lock);
    broker_sequence = ++g_sequence;
    if (broker_sequence == 0U) {
        broker_sequence = ++g_sequence;
    }
    tx_size = rpmsg_encode(request.type, broker_sequence, request.payload,
                           request.length, tx, sizeof(tx));
    if (tx_size == 0U ||
        write(g_device_fd, tx, tx_size) != (ssize_t)tx_size) {
        ++g_error_frames;
        monitor_emit("error", &request, request_data, request_size,
                     request.seq, broker_sequence, -1, "write_error");
        goto done;
    }
    ++g_tx_frames;
    g_tx_bytes += tx_size;
    transaction_start = monotonic_ms();
    monitor_emit("tx", &request, tx, tx_size, request.seq, broker_sequence,
                 -1, "ok");

    deadline = monotonic_ms() + TRANSACTION_TIMEOUT_MS;
    while (!g_stopping) {
        struct pollfd poll_fd = { .fd = g_device_fd, .events = POLLIN };
        int remaining = (int)(deadline - monotonic_ms());
        ssize_t rx_size;
        int ready;

        if (remaining <= 0) {
            errno = ETIMEDOUT;
            ++g_error_frames;
            monitor_emit("error", &request, tx, tx_size, request.seq,
                         broker_sequence, -1, "timeout");
            goto done;
        }
        ready = poll(&poll_fd, 1U, remaining);
        if (ready <= 0) {
            errno = ready == 0 ? ETIMEDOUT : errno;
            ++g_error_frames;
            monitor_emit("error", &request, tx, tx_size, request.seq,
                         broker_sequence, -1,
                         ready == 0 ? "timeout" : "poll_error");
            goto done;
        }
        rx_size = read(g_device_fd, rx, sizeof(rx));
        if (rx_size <= 0) {
            goto done;
        }
        if (!rpmsg_decode(rx, (size_t)rx_size, &response)) {
            ++g_error_frames;
            monitor_emit("drop", NULL, rx, (size_t)rx_size, request.seq,
                         0U, -1, "invalid_frame");
            continue;
        }
        if (response.seq != broker_sequence) {
            ++g_error_frames;
            monitor_emit("drop", &response, rx, (size_t)rx_size,
                         request.seq, response.seq, -1,
                         "sequence_mismatch");
            continue;
        }
        ++g_rx_frames;
        g_rx_bytes += (size_t)rx_size;
        monitor_emit("rx", &response, rx, (size_t)rx_size, request.seq,
                     response.seq, monotonic_ms() - transaction_start, "ok");
        *response_size = rpmsg_encode(
            response.type, request.seq, response.payload, response.length,
            response_data, response_capacity);
        if (*response_size == 0U) {
            errno = EMSGSIZE;
            goto done;
        }
        result = 0;
        break;
    }

done:
    pthread_mutex_unlock(&g_device_lock);
    return result;
}

static void *serve_client(void *argument)
{
    ClientContext *context = argument;
    uint8_t request[RPMSG_MAX_PAYLOAD + 5U];
    uint8_t response[RPMSG_MAX_PAYLOAD + 5U];
    int client_fd = context->client_fd;

    free(context);
    while (!g_stopping) {
        size_t response_size = 0U;
        ssize_t request_size = recv(client_fd, request, sizeof(request), 0);
        if (request_size == 0) {
            break;
        }
        if (request_size < 0) {
            if (errno == EINTR) {
                continue;
            }
            break;
        }
        if (transact(request, (size_t)request_size, response,
                     sizeof(response), &response_size) != 0 ||
            send(client_fd, response, response_size, MSG_NOSIGNAL) !=
                (ssize_t)response_size) {
            break;
        }
    }
    close(client_fd);
    return NULL;
}

static void usage(const char *program)
{
    fprintf(stderr, "Usage: %s [--device PATH] [--socket PATH] "
                    "[--monitor-socket PATH]\n", program);
}

int main(int argc, char **argv)
{
    const char *device = DEFAULT_DEVICE;
    const char *socket_path = RPMSG_BROKER_DEFAULT_SOCKET;
    const char *monitor_socket_path = RPMSG_BROKER_DEFAULT_MONITOR_SOCKET;
    struct sockaddr_un address;
    struct sockaddr_un monitor_address;
    pthread_t monitor_thread;
    int option;

    for (option = 1; option < argc; ++option) {
        if (strcmp(argv[option], "--device") == 0 && option + 1 < argc) {
            device = argv[++option];
        } else if (strcmp(argv[option], "--socket") == 0 &&
                   option + 1 < argc) {
            socket_path = argv[++option];
        } else if (strcmp(argv[option], "--monitor-socket") == 0 &&
                   option + 1 < argc) {
            monitor_socket_path = argv[++option];
        } else {
            usage(argv[0]);
            return 2;
        }
    }
    if (strlen(socket_path) >= sizeof(address.sun_path) ||
        strlen(monitor_socket_path) >= sizeof(monitor_address.sun_path)) {
        fprintf(stderr, "broker socket path is too long\n");
        return 2;
    }

    signal(SIGINT, request_stop);
    signal(SIGTERM, request_stop);
    signal(SIGPIPE, SIG_IGN);
    for (size_t index = 0; index < MAX_MONITOR_CLIENTS; ++index) {
        g_monitor_clients[index] = -1;
    }
    g_device_fd = open(device, O_RDWR | O_CLOEXEC);
    if (g_device_fd < 0) {
        perror("open RPMsg device");
        return 1;
    }
    g_server_fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    if (g_server_fd < 0) {
        perror("create broker socket");
        close(g_device_fd);
        return 1;
    }
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    memcpy(address.sun_path, socket_path, strlen(socket_path) + 1U);
    unlink(socket_path);
    if (bind(g_server_fd, (const struct sockaddr *)&address,
             sizeof(address)) != 0) {
        perror("bind broker socket");
        close(g_server_fd);
        close(g_device_fd);
        unlink(socket_path);
        return 1;
    }
    if (chmod(socket_path, 0660) != 0) {
        perror("set broker socket permissions");
        close(g_server_fd);
        close(g_device_fd);
        unlink(socket_path);
        return 1;
    }
    if (listen(g_server_fd, 16) != 0) {
        perror("listen on broker socket");
        close(g_server_fd);
        close(g_device_fd);
        unlink(socket_path);
        return 1;
    }

    g_monitor_server_fd = socket(AF_UNIX,
                                 SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    if (g_monitor_server_fd < 0) {
        perror("create monitor socket");
        request_stop(0);
        close(g_device_fd);
        unlink(socket_path);
        return 1;
    }
    memset(&monitor_address, 0, sizeof(monitor_address));
    monitor_address.sun_family = AF_UNIX;
    memcpy(monitor_address.sun_path, monitor_socket_path,
           strlen(monitor_socket_path) + 1U);
    unlink(monitor_socket_path);
    if (bind(g_monitor_server_fd,
             (const struct sockaddr *)&monitor_address,
             sizeof(monitor_address)) != 0 ||
        chmod(monitor_socket_path, 0660) != 0 ||
        listen(g_monitor_server_fd, MAX_MONITOR_CLIENTS) != 0) {
        perror("prepare monitor socket");
        request_stop(0);
        close(g_device_fd);
        unlink(socket_path);
        unlink(monitor_socket_path);
        return 1;
    }
    if (pthread_create(&monitor_thread, NULL, accept_monitor_clients, NULL) != 0) {
        perror("start monitor listener");
        request_stop(0);
        close(g_device_fd);
        unlink(socket_path);
        unlink(monitor_socket_path);
        return 1;
    }

    printf("rpmsg-broker %s: device=%s socket=%s monitor=%s\n",
           BROKER_VERSION, device, socket_path, monitor_socket_path);
    fflush(stdout);
    while (!g_stopping) {
        ClientContext *context;
        pthread_t thread;
        int client_fd = accept4(g_server_fd, NULL, NULL, SOCK_CLOEXEC);
        if (client_fd < 0) {
            if (errno == EINTR || g_stopping) {
                continue;
            }
            perror("accept broker client");
            break;
        }
        context = malloc(sizeof(*context));
        if (context == NULL) {
            close(client_fd);
            continue;
        }
        context->client_fd = client_fd;
        if (pthread_create(&thread, NULL, serve_client, context) != 0) {
            close(client_fd);
            free(context);
            continue;
        }
        pthread_detach(thread);
    }
    if (g_server_fd >= 0) {
        close(g_server_fd);
    }
    if (g_monitor_server_fd >= 0) {
        close(g_monitor_server_fd);
    }
    pthread_join(monitor_thread, NULL);
    pthread_mutex_lock(&g_monitor_lock);
    for (size_t index = 0; index < MAX_MONITOR_CLIENTS; ++index) {
        if (g_monitor_clients[index] >= 0) {
            close(g_monitor_clients[index]);
        }
    }
    pthread_mutex_unlock(&g_monitor_lock);
    close(g_device_fd);
    unlink(socket_path);
    unlink(monitor_socket_path);
    return 0;
}
