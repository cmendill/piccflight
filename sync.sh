#!/bin/bash

#sync files. use checksums. use gse time.
rsync -ctrlpgoDvz --delete --exclude-from=excludes.txt ../piccflight picture@picture:

