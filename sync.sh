#!/bin/bash

#set date & time on flight computer
ssh root@picture date -s @`( date -u +"%s" )`

#sync files
rsync -crlpgoDvz --delete --exclude-from=excludes.txt ../piccflight picture@picture:

#get generated settings files
rsync -avz picture@picture:piccflight/bin/output/settings/* bin/output/settings/
