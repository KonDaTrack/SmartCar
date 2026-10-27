# ADR 0004: GitHub for source, LAN for images

- Status: Accepted
- Date: 2026-10-25

## Context

The developer works alone. Images and bags are large; GitHub is suitable for
source, configs, scripts and docs, but not for multi-GB artifacts. The dev
machine and vehicle are on the same LAN when connected.

## Decision

- Back up **source/config/scripts/docs/manifests** to a private GitHub repo.
- Never commit images, bags, maps or `releases/`.
- Build the runtime image on the dev machine from a clean Git tag.
- Transfer the release bundle over the LAN with `rsync`; load it on the vehicle
  with `docker load`.
- Use immutable image tags; keep the current and previous release on the
  vehicle for rollback.
- No local registry initially. If deployment frequency increases, a local
  registry can be added without changing the image layout.

## Consequences

- The exact image can be rebuilt from the Git tag; the image tarball is kept on
  the dev machine/vehicle/external SSD.
- The vehicle does not need internet access or a compiler.
- The developer must maintain a separate backup for large images/bags (external
  SSD/NAS).
