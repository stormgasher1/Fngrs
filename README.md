# Fngrs
An all-in-one, vr hand glove with finger tracking for less than £50

## An example of what this project can do:
[![Fngrs Demo](https://img.youtube.com/vi/AnlR8ik4xHY/0.jpg)](https://www.youtube.com/watch?v=AnlR8ik4xHY)
## Credit
I would like to thank [@spayne](https://github.com/spayne) for his soft_knuckles example repo, which was a great help in setting up this project. 
## Status: Proof Of Concept
This was a weekend project that gained some interest, so I decided to publish this for people to view and perhaps contribute towards. I don't recommend people to actually build one for themselves yet, particularly as this project is no where near the level of polish that other vr controllers have, there's not many other controllers like this and that this is more of a **proof of concept** to perhaps gain the attention of companies that something like this is possible. Attempting to build one (at the moment) will most certainly be a waste of money. However, this project does "work".
## What is this?
This project is intended to provide affordable hand and finger tracking for as many headsets and games as possible. As such, the project is developed on OpenVR, making it compatible with SteamVR and all the headsets and controllers it supports. The OpenVR driver is compatible with all controllers that support OpenVR/SteamVR, as well as all games that support skeletal hand tracking through OpenVR.

## How does this work?
OpenVR provides a layer for drivers to interact with applications. The driver for this project passes hand tracking data from the glove into a new pair of controllers which can then be used to track finger movement from the arduinos and positioning and rotation from the controllers, without needing extra equipment for tracking.

## Current limitations of the project
* Hand positions doesn't perfectly follow the position of the hand in real-life, due to the slight offset the controller has on the hand to help the headset pick up the controllers with inside-out tracking, but I hope to fix this up soon with a much better approach than what is currently implemented.
* There is only a design for the Oculus Touch controllers (the ones which ship with the Rift S and Quest 1), but if someone creates a design of their own which suports other controllers, the driver and rest of the project should work fine with them.

### Requirements
* This project requires a 3D printer in order to make the mount for the components
* This project requires some knowledge in c++ for building the driver
* Soldering knowledge and electronic knowledge is required for building the glove
* <b>Again,</b>  I would like to re-iterate that this project is not something that I would recommend people to spend money on - this is more of a proof of concept device to show that cheap finger tracking is poosible. However, if you are willing to or have the components already available I would appreciate feedback on the project.

## How can I get started?
The GitHub Wiki contains instructions needed to get started with this project

## Contact me
If you run into any issues, please contact me on discord: `danwillm#8254`
