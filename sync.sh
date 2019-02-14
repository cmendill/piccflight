#!/bin/bash

#get generated settings files. use checksums. use gse time.
rsync -crlpgoDvz picture@picture:piccflight/bin/output/settings/* bin/output/settings/

#sync files. use checksums. use gse time.
rsync -ctrlpgoDvz --delete --exclude-from=excludes.txt ../piccflight picture@picture:

