#ifndef RPMSG_TRANSPORT_H
#define RPMSG_TRANSPORT_H

#define RPMSG_BROKER_DEFAULT_SOCKET "/run/rpmsg-broker/rpmsg.sock"
#define RPMSG_BROKER_DEFAULT_MONITOR_SOCKET "/run/rpmsg-broker/monitor.sock"

/*
 * Existing tools pass /dev/rpmsg0. That path is intentionally routed through
 * the broker. Use direct:/dev/rpmsg0 only after stopping rpmsg-broker.
 */
int rpmsg_transport_open(const char *endpoint);

#endif
