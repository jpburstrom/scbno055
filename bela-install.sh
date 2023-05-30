#!/bin/bash

wd=$(dirname $0)

scp -r $wd/Extensions/BNO root@bela.local:/root/.local/share/SuperCollider/Extensions/
