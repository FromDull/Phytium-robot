#define _POSIX_C_SOURCE 200809L

#include "rpmsg_transport.h"

#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/un.h>
#include <unistd.h>

int rpmsg_transport_open(const char *endpoint)
{
    const char *socket_path;
    struct sockaddr_un address;
    int fd;

    if (endpoint != NULL && strncmp(endpoint, "direct:", 7U) == 0) {
        return open(endpoint + 7, O_RDWR | O_CLOEXEC);
    }

    socket_path = getenv("RPMSG_BROKER_SOCKET");
    if (socket_path == NULL || socket_path[0] == '\0') {
        socket_path = RPMSG_BROKER_DEFAULT_SOCKET;
    }
    if (endpoint != NULL && strncmp(endpoint, "unix:", 5U) == 0) {
        socket_path = endpoint + 5;
    }
    if (strlen(socket_path) >= sizeof(address.sun_path)) {
        errno = ENAMETOOLONG;
        return -1;
    }

    fd = socket(AF_UNIX, SOCK_SEQPACKET | SOCK_CLOEXEC, 0);
    if (fd < 0) {
        return -1;
    }
    memset(&address, 0, sizeof(address));
    address.sun_family = AF_UNIX;
    memcpy(address.sun_path, socket_path, strlen(socket_path) + 1U);
    if (connect(fd, (const struct sockaddr *)&address, sizeof(address)) != 0) {
        int saved_errno = errno;
        close(fd);
        errno = saved_errno;
        return -1;
    }
    return fd;
}
