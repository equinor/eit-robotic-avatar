import { WebcamsEmpty } from "../services/webcams-test-tools";
import RoboticAvatar  from "./robotic-avatar"
import Video from "./video";

jest.mock('../services/webcams');

test('left is Video', () => {
    const robot = new RoboticAvatar(new WebcamsEmpty());
    expect(robot.left).toBeInstanceOf(Video)
})

test('right is Video', () => {
    const robot = new RoboticAvatar(new WebcamsEmpty());
    expect(robot.right).toBeInstanceOf(Video)
})