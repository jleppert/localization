#!/bin/bash

export HOME=/root
export NVM_DIR="$HOME/.nvm"
source "$NVM_DIR/nvm.sh" 

nvm use default

PM2=/root/.nvm/versions/node/v16.15.1/bin/pm2

export PM2_HOME=/var/log/pm2
$PM2 start /home/johnathan/devel/localization/startup.js

chmod 777 /var/log/pm2/pub.sock
chmod 777 /var/log/pm2/rpc.sock
