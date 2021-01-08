# Fngrs
An all-in-one, vr hand glove with finger tracking for less than £50

## An example of what this project can do:
[![Fngrs Demo](https://img.youtube.com/vi/AnlR8ik4xHY/0.jpg)](https://www.youtube.com/watch?v=AnlR8ik4xHY)
## Credit
I would like to thank [@spayne](https://github.com/spayne) for his soft_knuckles example repo, which was a great help in setting up this project. 
## Status: Proof Of Concept
This was a weekend project that gained some interest, so I decided to publish this for people to view and perhaps contribute towards. I don't recommend people to actually build one for themselves yet, but this is something which I would like to support (but will probably quite a long way down the line).
## What is this?
This project is intended to provide affordable hand and finger tracking for as many headsets and games as possible. As such, the project is developed on OpenVR, making it compatible with SteamVR and all the headset it supports. The OpenVR driver is compatible with all controllers that support OpenVR/SteamVR, as well as all games that support skeletal hand tracking through OpenVR.
## How does this work?
OpenVR provides a layer for drivers to interact with applications. The driver for this project passes hand tracking data from the glove into a new pair of controllers which can then be used to track finger movement from the arduinos and positioning and rotation from the controllers, without needing extra equipment for tracking.

## Current limitations of the project
Currently, hand positions are in the wrong positions compared to where they are in real-life. While there is some small fix to this, I have not yet implemeneted a system which is completley perfect in mapping the controller directly to the hand position.

Issues with the connection with the arduinos may arise, and may result in finger tracking stopping, requiring SteamVR to be restarted and arduinos to be reconnected.

### Requirements
* This project requires a 3D printer in order to make the mount for the components
* This project requires some knowledge in c++ for building the driver
* Soldering knowledge and electronic knowledge is required for building the glove
* <b>Again,</b>  I would like to re-iterate that this project is not something that I would recommend people to spend money on - this is more of a proof of concept device to show that cheap finger tracking is poosible. However, if you are willing to or have the components already available I would appreciate feedback on the project.

## How can I get started?
The GitHub Wiki contains instructions needed to get started with this project

## Contact me
If you run into any issues, please contact me on discord: `danwillm#8254`
