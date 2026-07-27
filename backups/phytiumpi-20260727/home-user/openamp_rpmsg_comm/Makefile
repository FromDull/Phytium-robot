CC ?= gcc
CFLAGS ?= -std=c99 -Wall -Wextra -I./src
PYTHON ?= python3
BUILD_DIR := build

.PHONY: all test test-python test-ros-python client logger gimbal broker firmware-elf install-rpmsg-broker install-rpmsg-monitor install-gimbal-daemon install-gimbal-tf-state install-chassis-state install-chassis-state-user clean

all: test client logger gimbal broker

$(BUILD_DIR):
	$(PYTHON) -c "import os; os.makedirs('$(BUILD_DIR)', exist_ok=True)"

test: $(BUILD_DIR) $(BUILD_DIR)/test_protocol $(BUILD_DIR)/test_motor_can \
	$(BUILD_DIR)/test_motor_balance_protocol $(BUILD_DIR)/test_lqr_controller \
	$(BUILD_DIR)/test_position_hold_controller \
	$(BUILD_DIR)/test_gimbal_controller $(BUILD_DIR)/test_servo_motion_controller \
	$(BUILD_DIR)/test_leg_joint_mapping $(BUILD_DIR)/test_leg_kinematics \
	$(BUILD_DIR)/rpmsg-broker test-python

test-python: $(BUILD_DIR)/rpmsg-broker
	$(PYTHON) -m unittest tests/test_chassis_state_bridge.py tests/test_gimbal_daemon.py tests/test_gimbal_tf_state_bridge.py tests/test_rpmsg_broker.py tests/test_rpmsg_monitor.py

test-ros-python:
	PYTHONPATH=ros2/gimbal_camera_tf_ros2:$${PYTHONPATH} $(PYTHON) -m unittest discover -s ros2/gimbal_camera_tf_ros2/test -p 'test_*.py'

client: $(BUILD_DIR) $(BUILD_DIR)/rpmsg_client

logger: $(BUILD_DIR) $(BUILD_DIR)/balance_logger

gimbal: $(BUILD_DIR) $(BUILD_DIR)/gimbal_test

broker: $(BUILD_DIR) $(BUILD_DIR)/rpmsg-broker

firmware-elf:
	bash ./tools/build_openamp_elf.sh

install-rpmsg-broker: $(BUILD_DIR)/rpmsg-broker
	install -d -m 0755 /usr/local/sbin
	install -m 0755 $(BUILD_DIR)/rpmsg-broker /usr/local/sbin/rpmsg-broker
	install -m 0644 integration/systemd/rpmsg-broker.service /etc/systemd/system/rpmsg-broker.service

install-rpmsg-monitor:
	install -d -m 0755 /usr/local/libexec /usr/local/share/rpmsg-monitor
	install -m 0755 linux_user/rpmsg_monitor.py /usr/local/libexec/rpmsg-monitor
	install -m 0644 linux_user/rpmsg_monitor.html /usr/local/share/rpmsg-monitor/index.html
	install -m 0644 integration/systemd/rpmsg-monitor.service /etc/systemd/system/rpmsg-monitor.service

install-gimbal-daemon:
	install -d -m 0755 /usr/local/libexec
	install -m 0755 linux_user/gimbal_daemon.py /usr/local/libexec/gimbal-daemon
	install -m 0755 linux_user/gimbalctl.py /usr/local/bin/gimbalctl
	install -m 0644 integration/systemd/gimbal-daemon.service /etc/systemd/system/gimbal-daemon.service

install-gimbal-tf-state:
	install -d -m 0755 /usr/local/libexec
	install -m 0755 linux_user/gimbal_tf_state_bridge.py /usr/local/libexec/gimbal-tf-state-bridge
	install -m 0644 integration/systemd/gimbal-tf-state.service /etc/systemd/system/gimbal-tf-state.service
	install -m 0644 integration/systemd/gimbal-camera-tf.service /etc/systemd/system/gimbal-camera-tf.service

install-chassis-state:
	install -d -m 0755 /usr/local/libexec
	install -m 0755 linux_user/chassis_state_bridge.py /usr/local/libexec/chassis-state-bridge
	install -m 0644 integration/systemd/chassis-state.service /etc/systemd/system/chassis-state.service

install-chassis-state-user:
	install -d -m 0755 $(HOME)/.config/systemd/user
	install -m 0644 integration/systemd/user/chassis-state.service $(HOME)/.config/systemd/user/chassis-state.service

$(BUILD_DIR)/test_protocol: src/rpmsg_protocol.c src/rpmsg_protocol.h tests/test_protocol.c
	$(CC) $(CFLAGS) $(filter %.c,$^) -o $@

$(BUILD_DIR)/test_motor_can: remote_firmware/motor_can.c remote_firmware/motor_can.h tests/test_motor_can.c
	$(CC) $(CFLAGS) -I./remote_firmware $(filter %.c,$^) -o $@

$(BUILD_DIR)/test_motor_balance_protocol: remote_firmware/motor_can.c remote_firmware/motor_can.h tests/motor_balance_protocol_test.c
	$(CC) $(CFLAGS) -I./remote_firmware $(filter %.c,$^) -o $@

$(BUILD_DIR)/test_lqr_controller: remote_firmware/lqr_controller.c remote_firmware/lqr_controller.h tests/lqr_controller_test.c
	$(CC) $(CFLAGS) -I./remote_firmware $(filter %.c,$^) -lm -o $@

$(BUILD_DIR)/test_position_hold_controller: remote_firmware/position_hold_controller.c remote_firmware/position_hold_controller.h tests/position_hold_controller_test.c
	$(CC) $(CFLAGS) -I./remote_firmware $(filter %.c,$^) -lm -o $@

$(BUILD_DIR)/test_gimbal_controller: remote_firmware/gimbal_controller.c remote_firmware/gimbal_controller.h remote_firmware/motor_can.c remote_firmware/motor_can.h tests/gimbal_controller_test.c tests/stubs/fgeneric_timer.h tests/stubs/fparameters.h
	$(CC) $(CFLAGS) -I./tests/stubs -I./remote_firmware $(filter %.c,$^) -o $@

$(BUILD_DIR)/test_servo_motion_controller: remote_firmware/servo_motion_controller.c remote_firmware/servo_motion_controller.h remote_firmware/phytium_servo_port.h tests/servo_motion_controller_test.c tests/stubs/fgeneric_timer.h
	$(CC) $(CFLAGS) -I./tests/stubs -I./remote_firmware $(filter %.c,$^) -o $@

$(BUILD_DIR)/test_leg_joint_mapping: src/leg_joint_mapping.c src/leg_joint_mapping.h tests/leg_joint_mapping_test.c
	$(CC) $(CFLAGS) $(filter %.c,$^) -o $@

$(BUILD_DIR)/test_leg_kinematics: src/leg_kinematics.c src/leg_kinematics.h src/leg_robot_geometry.c src/leg_robot_geometry.h tests/leg_kinematics_test.c
	$(CC) $(CFLAGS) $(filter %.c,$^) -lm -o $@

$(BUILD_DIR)/gimbal_test: src/rpmsg_protocol.c src/rpmsg_protocol.h src/rpmsg_transport.c src/rpmsg_transport.h linux_user/gimbal_test.c
	$(CC) $(CFLAGS) $(filter %.c,$^) -o $@

$(BUILD_DIR)/rpmsg_client: src/rpmsg_protocol.c src/rpmsg_protocol.h src/rpmsg_transport.c src/rpmsg_transport.h src/leg_joint_mapping.c src/leg_joint_mapping.h linux_user/rpmsg_client.c
	$(CC) $(CFLAGS) $(filter %.c,$^) -o $@

$(BUILD_DIR)/balance_logger: src/rpmsg_protocol.c src/rpmsg_protocol.h src/rpmsg_transport.c src/rpmsg_transport.h linux_user/balance_logger.c
	$(CC) $(CFLAGS) $(filter %.c,$^) -o $@

$(BUILD_DIR)/rpmsg-broker: src/rpmsg_protocol.c src/rpmsg_protocol.h src/rpmsg_transport.h linux_user/rpmsg_broker.c
	$(CC) $(CFLAGS) $(filter %.c,$^) -pthread -o $@

clean:
	$(PYTHON) -c "import shutil; shutil.rmtree('$(BUILD_DIR)', ignore_errors=True)"
