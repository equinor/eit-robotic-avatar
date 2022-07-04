import { hasUncaughtExceptionCaptureCallback } from "process";
import RoboticAvatar  from "./robot-avatar"
import Video from "./video";

test('left is Video', () => {
    const robot = new RoboticAvatar();
    expect(robot.left).toBeInstanceOf(Video)
})

test('right is Video', () => {
    const robot = new RoboticAvatar();
    expect(robot.right).toBeInstanceOf(Video)
})