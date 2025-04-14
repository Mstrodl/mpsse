FROM docker.io/debian:bookworm

RUN echo "deb http://deb.debian.org/debian bookworm-backports main" > /etc/apt/sources.list.d/backports.list && \
  apt-get update && \
  apt-get install -t bookworm-backports -y linux-headers-generic && \
  apt-get install build-essential -y

COPY . /app

WORKDIR /app/

RUN make -C /lib/modules/*/build M=/app/
