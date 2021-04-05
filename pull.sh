#!/bin/bash

#get generated settings files
rsync -acvz --update picture@picture:piccflight/bin/output/settings/* bin/output/settings/
