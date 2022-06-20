#!/bin/bash

#sync files
rsync -av --delete --exclude-from=excludes.txt ../piccflight picture@picture:

#make
ssh picture@picture 'source .bashrc; cd piccflight; make'


