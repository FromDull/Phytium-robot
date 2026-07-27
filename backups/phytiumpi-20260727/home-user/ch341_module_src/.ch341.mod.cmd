savedcmd_/home/user/ch341_module_src/ch341.mod := printf '%s\n'   ch341.o | awk '!x[$$0]++ { print("/home/user/ch341_module_src/"$$0) }' > /home/user/ch341_module_src/ch341.mod
