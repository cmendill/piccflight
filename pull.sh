#!/bin/bash

#get generated settings files
rsync -av --update picture@picture:piccflight/bin/output/settings/* bin/output/settings/
