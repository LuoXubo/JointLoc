#!/bin/bash
gnome-terminal --geometry 60x20+10+10 -- bash RelLoc/redis.sh & sleep 2     # run redis server
gnome-terminal --geometry 60x20+10+10 -- bash AbsLoc/server.sh & sleep 2    # publish images to redis
gnome-terminal --geometry 60x20+10+10 -- bash RelLoc/relloc.sh & sleep 2    # run relative localization
gnome-terminal --geometry 60x20+10+10 -- bash AbsLoc/absloc.sh & sleep 2    # run absolute localization