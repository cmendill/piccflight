#!/bin/bash

rsync -crlpgoDvz --delete --exclude-from=excludes.txt ../piccflight picture@lowfs:


