# Docker Permission Denied Fix on Jetson

> Date: 2026-02-12
> Status: Resolved
> Keywords: docker, permission denied, group, socket, jetson

## Problem

After installing Docker and running `sudo usermod -aG docker georgejetson`, the user still gets `permission denied` errors when running `docker ps` without sudo:

```
Got permission denied while trying to connect to the Docker daemon socket at unix:///var/run/docker.sock
```

## Root Cause

`usermod -aG docker $USER` modifies `/etc/group` but the **current shell session** doesn't pick up the new group membership until you log out and back in. This is a Linux group membership behavior, not a Docker bug.

## Solutions

### Quick Fix (current session only)
```bash
newgrp docker
```
This starts a new shell with the `docker` group active. You'll need to run this in every terminal you open until you fully log out/in.

### Permanent Fix (recommended)
Log out of the Jetson completely (GUI session too if on desktop), then log back in. All new sessions will have the `docker` group.

### Verify
```bash
# Check group membership
id | grep docker

# Check Docker socket permissions
ls -la /var/run/docker.sock
# Should show: srw-rw---- root docker

# Test without sudo
docker ps
docker compose version
```

## What NOT to Do

- **Don't `chmod 777 /var/run/docker.sock`** — this is a security risk
- **Don't install a second Docker** via snap or pip — JetPack ships Docker, use it
- **Don't use the x86_64 compose binary on Jetson** — use `docker-compose-linux-aarch64` (ARM64)
  - The command `curl -SL https://github.com/docker/compose/releases/download/v2.35.0/docker-compose-linux-x86_64` will download a binary that **cannot execute** on Jetson (aarch64)

## Jetson-Specific Notes

- JetPack R36.4.4 ships Docker 28.2.2 and Docker Compose v2.36.2 pre-installed
- NVIDIA Container Toolkit 1.16.2 is pre-installed
- `/etc/docker/daemon.json` needs `"default-runtime": "nvidia"` for GPU containers
- Ollama systemd override at `/etc/systemd/system/ollama.service.d/override.conf` sets `OLLAMA_HOST=0.0.0.0`

## Configuration Files

### /etc/docker/daemon.json
```json
{
    "runtimes": {
        "nvidia": {
            "args": [],
            "path": "nvidia-container-runtime"
        }
    },
    "default-runtime": "nvidia"
}
```

### Docker group
```bash
# User should be in docker group
getent group docker
# docker:x:995:georgejetson
```

---

*See also: [jetson-system-inventory.md](./jetson-system-inventory.md)*
