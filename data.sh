#!/bin/bash

#get generated settings files
rsync -avz --update picture@picture:piccflight/bin/output/settings/* bin/output/settings/
