#!/bin/bash
rm -f abs_res.txt
rm -f log.txt

python server.py && python estimate.py --terrain=Orbit