import { WebcamsDevice } from "../services/webcams-test-tools";
import Video from "./video";

test('default values', async () => {
    const video = new Video(new WebcamsDevice());
    expect(video.show_local).toBe(false);
    expect(await video.getSources()).toStrictEqual([]);
})

test('default show sources', async () => {
    const video = new Video(new WebcamsDevice(
        videoDevice("12345678", "TestCam1" )
    ));
    video.show_local = true;

    expect(video.show_local).toBe(true);
    const videoSources = [{
        id: "12345678",
        type: "local",
        label: "TestCam1 (123456)"
    }]
    expect(await video.getSources()).toStrictEqual(videoSources);
})

function videoDevice(id: string, label: string): MediaDeviceInfo {
    return {
        deviceId: id,
        groupId: id,
        kind: "videoinput",
        label: label,
        toJSON() {}
    }
}