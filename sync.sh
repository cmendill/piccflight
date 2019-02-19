#!/bin/bash

#sync files
rsync -avz --update --delete --exclude-from=excludes.txt ../piccflight picture@picture:

