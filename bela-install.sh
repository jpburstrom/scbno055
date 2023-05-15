#!/bin/bash

wd=$(dirname $0)

scp -r $wd/Extensions/BNO root@bela:/root/.local/share/SuperCollider/Extensions/
