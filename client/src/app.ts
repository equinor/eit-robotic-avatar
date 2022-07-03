import Status from "./status";
import Video from "./video";

export default class RoboticAvatar{
    left_video: Video;
    right_video: Video;

    constructor(private elem: HTMLElement) {
        const left_video_status = new Status(this.elem.querySelector("#left_video_status")!);
        this.left_video = new Video(this.elem.querySelector("#left_video")!, left_video_status);

        const right_video_status = new Status(this.elem.querySelector("#right_video_status")!);
        this.right_video = new Video(this.elem.querySelector("#right_video")!, right_video_status)
    }
}