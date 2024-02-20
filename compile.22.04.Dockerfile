ARG UBUNTU_VERSION=22.04

FROM ubuntu:${UBUNTU_VERSION}

# Specify the Rust version
ARG RUST_VERSION=1.73

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies and Rust
RUN apt-get update && apt-get install -y curl build-essential libssl-dev pkg-config libclang-dev libproj-dev
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"
RUN rustup install ${RUST_VERSION} && rustup default ${RUST_VERSION}