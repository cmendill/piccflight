#!/bin/bash

#sync files
rsync -acvz --update --delete --exclude-from=excludes.txt ../piccflight picture@picture:

