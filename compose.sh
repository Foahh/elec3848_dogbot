#!/bin/bash
if [ "$(arch)" == "x86_64" ];
then
    cd amd64/ || exit
elif [ "$(arch)" == "aarch64" ]
then
    cd arm64/ || exit
fi

docker compose build || exit
docker compose up