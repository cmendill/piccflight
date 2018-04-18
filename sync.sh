#!/bin/bash

rsync -crlpgoDvz --delete --exclude-from=excludes.txt ./ picture@lowfs:piccflight/working


