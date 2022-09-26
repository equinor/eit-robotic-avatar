# EIT Robotic Avatar
Move humans into robots.

## Archived 
The development of robotics avatars have move to a new repository: https://github.com/equinor/eit-robotic-avatar-two . 


## License and contribution
This project is licensed under the MIT License se [LICENSE](./LICENSE) file.

This repo hosts an experiment and will be archived after the trails are concluded. So we are not looking for outside contributions. If you find this repo interesting feel free to fork.

## Usage

`cargo run` to start the main server application.
`yarn parcel watch` start the web client.


## Learnings

* We will need to keep the relative distance between eyes and limbs.
    * When i tried to pick up an item using the camera eyes. I missed by about the same distance the cameras was from my eyes.
* But we can play around with scale. 
    * When hold the camera low i believe i was a meter high.
    * When i keeps my camera high i was a giant in a big room.
* The brain seems to handle lag well enough.
    * Only had problems moving when the latency became grater that 1 second.
* We had put the cameras at a slight angle to not see double.
    * It seems like eyes changes angle from near to far.
