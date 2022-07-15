# See here for image contents: https://github.com/microsoft/vscode-dev-containers/blob/v0.236.0/containers/rust/.devcontainer/base.Dockerfile

# [Choice] Debian OS version (use bullseye on local arm64/Apple Silicon): buster, bullseye
ARG VARIANT="bullseye"
FROM mcr.microsoft.com/vscode/devcontainers/rust:1-${VARIANT}

# Install wasm-pack
RUN curl https://rustwasm.github.io/wasm-pack/installer/init.sh -sSf | sh

# Old python demo dependensees.
RUN apt-get update && export DEBIAN_FRONTEND=noninteractive \
     && apt-get -y install --no-install-recommends pipx python3-venv
