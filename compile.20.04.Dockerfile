ARG UBUNTU_VERSION=20.04

FROM ubuntu:${UBUNTU_VERSION}

# Specify the Rust version
ARG RUST_VERSION=1.73

# Avoid prompts from apt
ENV DEBIAN_FRONTEND=noninteractive

# Install build dependencies and Rust
RUN apt-get update && apt-get install -y curl build-essential libssl-dev pkg-config libclang-dev libproj-dev proj-bin
RUN curl --proto '=https' --tlsv1.2 -sSf https://sh.rustup.rs | sh -s -- -y
ENV PATH="/root/.cargo/bin:${PATH}"
COPY ./src/ /home/generator-mavlink-plan/src
COPY ./Cargo.lock /home/generator-mavlink-plan/Cargo.lock
COPY ./Cargo.toml /home/generator-mavlink-plan/Cargo.toml
# RUN apt-get update && apt-get install -y musl-tools && rustup target add x86_64-unknown-linux-musl