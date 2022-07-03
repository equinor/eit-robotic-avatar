//@ts-check
import Status from "./status.mjs";
import Video from "./video.mjs";

export default class RoboticAvatar{
    /**
     * @param {HTMLElement} app_elem
     */
    constructor(app_elem) {
        this.elem = app_elem;
        // @ts-ignore
        const left_video_status = new Status(this.elem.querySelector("#left_video_status"));
        this.left_video = new Video(this.elem.querySelector("#left_video"), left_video_status);

        
        // @ts-ignore
        const right_video_status = new Status(this.elem.querySelector("#right_video_status"));
        this.right_video = new Video(this.elem.querySelector("#right_video"), right_video_status)
    }
}