# eit-robotic-avatar
Move humans into robots. 

## Usage

`cargo run` to start the main server application.


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
