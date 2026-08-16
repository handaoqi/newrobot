# 3588 Charge Pile Vendor Bundle

This directory is a byte-for-byte backup of the charge-pile vendor bundle
deployed on the RK3588 board at:

`/home/firefly/charge_pile_v1.0.3b/charge_pile_xg_lib_v1.0.3b/`

The active controller is `systemd/roamerx-charge-pile.service`, which launches
the repository script `script/robot/charge_pile_controller.sh`. That script
selects one of the `dog_send_three_states/` executables using the state file.

The binaries are ARM64 vendor artifacts. Keep their file modes intact when
deploying them to the 3588 board.
