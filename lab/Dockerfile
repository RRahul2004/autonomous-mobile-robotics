# syntax=docker/dockerfile:1

FROM ubuntu:22.04

# ---------------------------------------------------------
# Environment
# ---------------------------------------------------------

ENV DEBIAN_FRONTEND=noninteractive
ENV TZ=America/Toronto

# ---------------------------------------------------------
# Basic packages required during setup
# ---------------------------------------------------------

RUN apt-get update && \
    apt-get install -y \
        sudo \
        openssh-client \
        ca-certificates \
        lsb-release \
        lsb-core \
        tzdata \
        apt-utils && \
    ln -fs /usr/share/zoneinfo/$TZ /etc/localtime && \
    dpkg-reconfigure --frontend noninteractive tzdata && \
    rm -rf /var/lib/apt/lists/*

# ---------------------------------------------------------
# Create user
# ---------------------------------------------------------

RUN useradd -m -s /bin/bash rahul && \
    echo "rahul ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/rahul && \
    chmod 0440 /etc/sudoers.d/rahul

# ---------------------------------------------------------
# Configure SSH for Waterloo GitLab
# ---------------------------------------------------------

RUN mkdir -p -m 700 /home/rahul/.ssh && \
    ssh-keyscan git.uwaterloo.ca >> /home/rahul/.ssh/known_hosts && \
    chown -R rahul:rahul /home/rahul/.ssh

RUN printf '%s\n' \
    'Host gitlab-school' \
    '    HostName git.uwaterloo.ca' \
    '    User ist-git' \
    > /home/rahul/.ssh/config && \
    chown rahul:rahul /home/rahul/.ssh/config && \
    chmod 600 /home/rahul/.ssh/config

# ---------------------------------------------------------
# Switch to normal user
# ---------------------------------------------------------

USER rahul

WORKDIR /home/rahul

# ---------------------------------------------------------
# Copy setup script
# ---------------------------------------------------------

COPY --chown=rahul:rahul setup.sh /home/rahul/setup.sh

RUN chmod +x /home/rahul/setup.sh

# ---------------------------------------------------------
# Test SSH agent forwarding
#
# uid=1000/gid=1000 makes the BuildKit SSH socket
# accessible to the rahul user.
# ---------------------------------------------------------

RUN --mount=type=ssh,uid=1000,gid=1000 \
    ssh -T -o BatchMode=yes ist-git@git.uwaterloo.ca

# ---------------------------------------------------------
# Run setup
# ---------------------------------------------------------

RUN --mount=type=ssh,uid=1000,gid=1000 \
    /home/rahul/setup.sh

# ---------------------------------------------------------
# Start shell
# ---------------------------------------------------------

CMD ["bash", "-c", "cd /ros2_ws && exec bash"]
