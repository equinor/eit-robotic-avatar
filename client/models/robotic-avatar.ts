import { IWebcams } from "../services/webcams";
import Video from "./video";

export default class RoboticAvatar {
    public readonly left: Video;
    public readonly right: Video;


    constructor(webcams: IWebcams) {
        this.left = new Video(webcams);
        this.right = new Video(webcams);
    }
}