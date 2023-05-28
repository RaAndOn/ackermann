#!/bin/bash
set -e
git pull origin $(git branch --show-current)
docker-compose down
docker-compose build
docker-compose up -d