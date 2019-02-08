#!/bin/bash

#get generated settings files
rsync -avz picture@picture:piccflight/bin/output/settings/* bin/output/settings/

#sync files
rsync -avz --delete --exclude-from=excludes.txt ../piccflight picture@picture:

