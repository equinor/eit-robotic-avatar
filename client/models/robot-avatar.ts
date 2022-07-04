import Video from "./video";

export default class RoboticAvatar {
    public readonly left: Video;
    public readonly right: Video;
    constructor() {
        this.left = new Video();
        this.right = new Video();
    }
}