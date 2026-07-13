ARG UBUNTU_DISTRO="22.04"

FROM ubuntu:${UBUNTU_DISTRO}

# 【新增】在更新和安装软件之前，将官方源替换为清华大学镜像源，并清除代理变量以避免 403 错误
RUN sed -i 's@http://archive.ubuntu.com/ubuntu/@http://mirrors.tuna.tsinghua.edu.cn/ubuntu/@g' /etc/apt/sources.list && \
    sed -i 's@http://security.ubuntu.com/ubuntu/@http://mirrors.tuna.tsinghua.edu.cn/ubuntu/@g' /etc/apt/sources.list && \
    unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY && \
    packages='libsdl2-2.0 xserver-xorg libvulkan1 libomp5' \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y $packages \
    && rm -rf /var/lib/apt/lists/*

# Install the `xdg-user-dir` tool so the Unreal Engine can use it to locate the user's Documents directory
RUN unset http_proxy https_proxy HTTP_PROXY HTTPS_PROXY && \
    packages='xdg-user-dirs' \
    && apt-get update \
    && DEBIAN_FRONTEND=noninteractive apt-get install -y $packages \
    && rm -rf /var/lib/apt/lists/*

RUN useradd -m carla

WORKDIR /workspace
COPY --chown=carla:carla . .

USER carla

ENV NVIDIA_VISIBLE_DEVICES=all
ENV NVIDIA_DRIVER_CAPABILITIES=all
ENV SDL_VIDEODRIVER="x11"

# You can also run CARLA in offscreen mode with -RenderOffScreen
CMD ["/bin/bash", "CarlaUnreal.sh"]