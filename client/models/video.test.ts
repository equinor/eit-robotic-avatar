import Webcams from "../services/webcams";
import Video from "./video";

test('default values', async () => {
    const video = new Video(new Webcams());
    expect(video.show_local).toBe(false);
    expect(await video.getSources()).toStrictEqual([]);
})

