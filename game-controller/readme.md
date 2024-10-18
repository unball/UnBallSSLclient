# UnBallSSLclient Game Controller

This project is a client for the SSL Game Controller.

## Overview

The SSL Game Controller is used in the RoboCup Small Size League (SSL) to manage and control the game state. This client interacts with the game controller to receive and send game state information.

## Repository

For more details, please visit the [SSL Game Controller Commands](https://github.com/RoboCup-SSL/ssl-game-controller/tree/master/cmd).

## To compile proto files
```sh
cd proto
protoc --python_out=. *.proto
```