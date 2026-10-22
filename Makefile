SHELL := /bin/bash

.PHONY: vm-provision vm-up vm-down vm-ssh vm-status vm-snapshot

vm-provision:
	./infra/provision.sh

vm-up:
	./infra/vm-start.sh

vm-down:
	./infra/vm-stop.sh

vm-ssh:
	./infra/vm-ssh.sh

vm-status:
	./infra/vm-status.sh

vm-snapshot:
	./infra/snapshot.sh
