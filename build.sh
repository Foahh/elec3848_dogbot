#!/bin/bash
if [ "$(arch)" == "x86_64" ];
then
    cd amd64/ || exit 1
elif [ "$(arch)" == "aarch64" ]
then
    cd arm64/ || exit 1
fi

docker compose build
docker compose up -d dev