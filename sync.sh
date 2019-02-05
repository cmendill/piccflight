#!/bin/bash

rsync -avz --delete --exclude-from=excludes.txt ../piccflight picture@picture:


