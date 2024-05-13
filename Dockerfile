FROM docker.io/debian:bookworm

RUN apt-get update && apt-get install build-essential linux-headers-generic -y

COPY . /app

WORKDIR /app/

RUN make -C /lib/modules/*/build M=/app/
