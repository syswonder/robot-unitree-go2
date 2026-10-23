# Hub / Console installation and directory discovery

The Go2 package sources are in this repository. The public robot entry in
Robonix's catalog points here; it is not an image of a prebuilt robot computer.
Uploading a Console deployment backup transfers manifests/versions, not local
package source directories, models, maps, ROS installations or compiled outputs.

## If Console shows “directory not found”

Older manifests used `${ROBONIX_DEPLOY_DIR}/packages/...` for local packages.
That variable was exported by this repository's shell launcher, but a Console
catalog download does not necessarily run that launcher. An unset variable
therefore makes existing packages appear missing. The root manifest now uses
`./packages/...`, `./third_party/...` and `./soma.yaml`, resolved against the
manifest directory by rbnx. `ROBONIX_SOURCE_PATH` for Speech refers to Robonix's
own installation, not the Go2 checkout; do not replace it with `./services/speech`.

Mapping, Navigation, Unitree SDK2 and Unitree ROS2 are pinned Git submodules.
A GitHub source ZIP omits their contents. For a Git checkout, initialize them:

```bash
git clone --recurse-submodules https://github.com/syswonder/robot-unitree-go2.git
cd robot-unitree-go2
# Also needed if the repository was already downloaded without submodules:
git submodule update --init --recursive
```

There are seven repository-local package entries, one URL-fetched audio bridge
and Robonix's built-in Speech service. “Not downloaded” for the audio bridge is
different from “directory not found”; it is fetched by the normal package build.
Do not copy only the YAML into an empty directory: relative paths need the
repository files next to it.

## Runtime paths and machine configuration

Relative package paths fix discovery. Runtime cache/map paths also used the
launcher variable, and providers execute from different working directories.
For Console, render a separate machine-local manifest from the checkout:

```bash
mkdir -p console-local
python3 scripts/prepare_console_manifest.py --output console-local/robonix_manifest.yaml
```

This uses the Python/PyYAML environment supplied by Robonix, reads only the public
manifest, and creates `console-local/robonix_manifest.yaml`. It resolves deployment
paths and defines `ROBONIX_DEPLOY_DIR` for package scripts. It does not read
`.env`, install dependencies, build packages, or start hardware. It refuses to
overwrite an existing file; use `--output another-manifest.yaml` for a new copy.
In Console's deployment management, create a deployment with the absolute path
to `console-local` as its directory. Re-render after moving the
checkout to another machine or path. Do not publish its machine-specific paths
as the public Hub template or set a global deployment directory shared by
unrelated Console deployments.

With no arguments, the renderer instead creates `robonix_manifest.console.yaml`
next to the original manifest, useful for `rbnx ... -f` or manual import.

Configure the remaining machine variables from `.env.example` in Console. A
Console boot is not guaranteed to source `.env`: network interface, sensor
placement, map ID/mode, landmark path, model endpoints and Unitree ROS2 overlay
must describe the destination host. Keep the existing motion-disabled defaults.
ROS 2 Humble, compatible Robonix/codegen dependencies, the Unitree message
overlay, Docker/native backend dependencies and speech models still need to be
prepared before a full build/boot. See [DEPLOYMENT.md](DEPLOYMENT.md).

This path fix is not an ARM/Jetson build or physical acceptance result. A
working x86 workstation build cannot be copied as Jetson executables. Start
from source (including submodules), build for the target architecture, and
retain the normal read-only and supervised validation sequence.

## Publishing and refreshing

The catalog already points to this repository, so no extra catalog entry is
needed for these bundled packages. Once the source fix is merged, a catalog
refresh must index the new source revision; verify the displayed revision.
Existing Console downloads/backups may retain old manifests. Re-download or
apply the reviewed path patch to the existing checkout, initialize submodules,
then regenerate its local Console manifest. A backup of a local deployment is
not a portable installer.
