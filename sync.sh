#!/bin/bash

rsync -crlpgoDvz --delete --exclude-from=excludes.txt ./ picture@jpl:wcs/working/
rsync -crlpgoDvz --delete --exclude-from=excludes.txt ../data/ picture@jpl:wcs/data/

