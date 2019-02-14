#!/bin/bash

#get generated settings files
rsync -acvz picture@picture:piccflight/bin/output/settings/* bin/output/settings/

#sync files
rsync -acvz --delete --exclude-from=excludes.txt ../piccflight picture@picture:

