# Deploy

## Vehicle prerequisites (one-time)

Ubuntu 18.04 on `yunle@192.168.100.10`:

```bash
sudo apt-get update
sudo apt-get install -y docker.io rsync zstd
sudo usermod -aG docker yunle
# log out / log in for the docker group to take effect
docker --version
```

Ubuntu 18.04 does not ship the Docker Compose v2 plugin. Download it on the dev
machine and copy it over:

```bash
# on the dev machine (has internet)
curl -SL \
  https://github.com/docker/compose/releases/download/v2.29.7/docker-compose-linux-x86_64 \
  -o /tmp/docker-compose
scp /tmp/docker-compose yunle@192.168.100.10:/tmp/docker-compose

# on the vehicle
sudo mkdir -p /usr/local/lib/docker/cli-plugins
sudo mv /tmp/docker-compose /usr/local/lib/docker/cli-plugins/docker-compose
sudo chmod +x /usr/local/lib/docker/cli-plugins/docker-compose
docker compose version
```

If the vehicle is offline, download the plugin and apt packages on the dev
machine and copy them over. Docker 20.10 can load images built by Docker 29.

## Release bundle

`./scripts/release.sh v0.1.0` produces a self-contained directory. The vehicle
receives only this bundle; it never receives source or a compiler.

```text
releases/v0.1.0/
├── image.tar.zst
├── image.sha256
├── compose.vehicle.yaml
├── config/
├── rollback.sh
├── healthcheck.sh
├── manifest.yaml
└── .env
```

## Deploy

```bash
./scripts/deploy.sh v0.1.0
```

This checks the vehicle for `docker`, `rsync`, `zstd` and `docker compose`, then:

1. rsyncs the release to `/opt/smartcar/releases/v0.1.0`;
2. verifies `image.sha256`;
3. decompresses and `docker load`s the image;
4. points `/opt/smartcar/current` at the release;
5. runs `docker compose -f compose.vehicle.yaml up -d`.

The vehicle starts `sensors` and `slam`. The `chassis` and `vehicle` (combined)
services are opt-in profiles.

## Health

```bash
./scripts/health.sh
# or on the vehicle:
cd /opt/smartcar/current && ./healthcheck.sh
```

The skeleton healthcheck prints compose status, topic lists and container stats.
Replace the TODOs with real `ros2 topic hz` checks once drivers exist.

## Rollback

```bash
./scripts/rollback.sh
```

This switches `current` to the previous release directory and runs
`docker compose up -d`. It requires at least two release directories.

## Configuration changes

The release contains a copy of `config/`. For temporary on-vehicle tuning:

```bash
cd /opt/smartcar/current
# edit config/*.yaml
docker compose -f compose.vehicle.yaml up -d --force-recreate
```

For permanent changes, edit `config/` on the dev machine, commit, create a new
release and deploy. Do not leave long-term changes only on the vehicle.

## Autostart

```bash
sudo cp deploy/systemd/smartcar.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now smartcar.service
```

The unit runs `docker compose up` in `/opt/smartcar/current` and stops it on
shutdown.

## Disk hygiene

The runtime image is typically 2–4 GB compressed to 1–2 GB. Keep current and
previous releases, then prune:

```bash
docker image prune -f
docker system df
```
