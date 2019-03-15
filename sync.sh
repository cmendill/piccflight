#!/bin/bash

#sync files
rsync -crlpgoDvz --delete --exclude-from=excludes.txt ../piccflight picture@picture:


