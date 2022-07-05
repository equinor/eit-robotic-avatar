import Webcams from "../services/webcams";
import RoboticAvatar  from "./robotic-avatar"
import Video from "./video";

jest.mock('../services/webcams');

test('left is Video', () => {
    const robot = new RoboticAvatar(new Webcams());
    expect(robot.left).toBeInstanceOf(Video)
})

test('right is Video', () => {
    const robot = new RoboticAvatar(new Webcams());
    expect(robot.right).toBeInstanceOf(Video)
})